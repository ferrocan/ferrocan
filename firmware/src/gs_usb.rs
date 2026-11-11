// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The `gs_usb` device: the control requests, and the bridge between the bulk
//! endpoints and the CAN channel.
//!
//! The host's frames go to the channel as they come, while it has room; the
//! endpoint is not read otherwise, so USB holds the host back. Each is echoed
//! to the host when the channel reports it, so an echo means the frame was
//! acknowledged on the bus, not merely queued. A frame aborted or given up is
//! echoed too, as Linux's `gs_usb` frees a transmit slot only on an echo; it
//! has no way to be told a frame failed.

use core::cell::RefCell;
use core::future::pending as never;
use core::sync::atomic::{AtomicBool, Ordering};

use can_hal::{BusState, BusStatus, CanFrame, CanId, ErrorCounters, Event, Frame, Outcome, TransmitId};
use defmt::{debug, info, warn};
use embassy_futures::select::{Either, Either4, select, select4};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::Handler;
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{EndpointError, EndpointIn, EndpointOut};
use gs_usb_protocol::{BtConst, CLASSIC_TS_LEN, DeviceBittiming, DeviceConfig, DeviceMode, DeviceState, ECHO_ID_RX, HostFrame, Request as GsRequest, can_err, can_id, decode_u32, feature, frame_flag, identify, mode, state};

use crate::channel::{BusError, Channel, Fault, Mode, Settings, Timing};

/// The features the device offers.
pub const FEATURES: u32 = MODE_FLAGS | feature::IDENTIFY | feature::GET_STATE;

/// The flags the host may ask for when starting the channel: the features
/// that are modes.
const MODE_FLAGS: u32 = feature::LISTEN_ONLY | feature::LOOP_BACK | feature::ONE_SHOT | feature::HW_TIMESTAMP | feature::BERR_REPORTING;

/// The error counters' warning level.
const WARNING_LEVEL: u8 = 96;

/// The firmware's version, as `DEVICE_CONFIG` reports it. At least 2: Linux's
/// `gs_usb` ignores the identify feature of firmware older than that, as
/// candleLight's first releases had it broken.
const SW_VERSION: u32 = 2;
/// The hardware's version.
const HW_VERSION: u32 = 1;

/// What the host asked the channel to do.
pub enum Command {
    /// Join the bus with these settings, stamping frames or not.
    Start { settings: Settings, timestamps: bool },
    /// Leave the bus.
    Reset,
}

/// What the control handler and the bridge share.
pub struct Shared {
    timing: Mutex<CriticalSectionRawMutex, RefCell<Option<Timing>>>,
    command: Signal<CriticalSectionRawMutex, Command>,
    /// Whether the channel is on the bus, as the bridge last left it.
    running: AtomicBool,
    /// Whether the host asked the device to identify itself.
    identify: AtomicBool,
    /// A frame was received since the LEDs last looked.
    rx_activity: AtomicBool,
    /// A frame was sent since the LEDs last looked.
    tx_activity: AtomicBool,
}

impl Shared {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            timing: Mutex::new(RefCell::new(None)),
            command: Signal::new(),
            running: AtomicBool::new(false),
            identify: AtomicBool::new(false),
            rx_activity: AtomicBool::new(false),
            tx_activity: AtomicBool::new(false),
        }
    }

    /// Whether the channel is on the bus.
    pub fn running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    /// Whether a frame was received since the last call. (The Cortex-M0 has
    /// no atomic swap: a frame between the load and the store goes unseen,
    /// and its blink with it.)
    pub fn take_rx_activity(&self) -> bool {
        let seen = self.rx_activity.load(Ordering::Relaxed);
        self.rx_activity.store(false, Ordering::Relaxed);
        seen
    }

    /// Whether a frame was sent since the last call.
    pub fn take_tx_activity(&self) -> bool {
        let seen = self.tx_activity.load(Ordering::Relaxed);
        self.tx_activity.store(false, Ordering::Relaxed);
        seen
    }

    /// Whether the host asked the device to identify itself: blink, say.
    pub fn identifying(&self) -> bool {
        self.identify.load(Ordering::Relaxed)
    }
}

impl Default for Shared {
    fn default() -> Self {
        Self::new()
    }
}

/// The one channel's shared state.
pub static SHARED: Shared = Shared::new();

/// Answers the `gs_usb` vendor requests to interface 0, for a channel of type
/// `C`.
pub struct Control<C: Channel> {
    shared: &'static Shared,
    status: C::Status,
}

impl<C: Channel> Control<C> {
    /// A handler sharing `shared` with the bridge, which reads the channel's
    /// error state from `status`.
    #[must_use]
    pub const fn new(shared: &'static Shared, status: C::Status) -> Self {
        Self { shared, status }
    }

    /// The channel's state, as `GET_STATE` reports it.
    fn state(&self) -> DeviceState {
        let counters = self.status.error_counters().unwrap_or(ErrorCounters { transmit: 0, receive: 0 });
        let state = if self.shared.running.load(Ordering::Relaxed) {
            match self.status.bus_state() {
                Ok(BusState::BusOff) => state::BUS_OFF,
                Ok(BusState::ErrorPassive) => state::ERROR_PASSIVE,
                _ if counters.transmit >= WARNING_LEVEL || counters.receive >= WARNING_LEVEL => state::ERROR_WARNING,
                _ => state::ERROR_ACTIVE,
            }
        } else {
            state::STOPPED
        };
        DeviceState {
            state,
            rxerr: u32::from(counters.receive),
            txerr: u32::from(counters.transmit),
        }
    }

    fn start(&self, flags: u32) -> OutResponse {
        if flags & !MODE_FLAGS != 0 {
            warn!("gs_usb: mode flags {:#x} not offered", flags);
            return OutResponse::Rejected;
        }
        let Some(timing) = self.shared.timing.lock(|t| *t.borrow()) else {
            warn!("gs_usb: start before bit timing");
            return OutResponse::Rejected;
        };
        let mode = match (flags & feature::LISTEN_ONLY != 0, flags & feature::LOOP_BACK != 0) {
            (false, false) => Mode::Normal,
            (true, false) => Mode::Silent,
            (false, true) => Mode::Loopback,
            (true, true) => Mode::SilentLoopback,
        };
        let settings = Settings {
            nominal: timing,
            data: None,
            mode,
            retransmit: flags & feature::ONE_SHOT == 0,
            bus_errors: flags & feature::BERR_REPORTING != 0,
        };
        self.shared.command.signal(Command::Start {
            settings,
            timestamps: flags & feature::HW_TIMESTAMP != 0,
        });
        OutResponse::Accepted
    }
}

/// Whether `req` is for this device: a vendor request to interface 0.
fn is_gs_usb(req: &Request) -> bool {
    req.request_type == RequestType::Vendor && req.recipient == Recipient::Interface && req.index == 0
}

impl<C: Channel> Handler for Control<C> {
    /// On USB suspend the channel leaves the bus, as candleLight's does: the
    /// host can't take its frames. Linux re-probes the device on resume.
    fn suspended(&mut self, suspended: bool) {
        if suspended {
            info!("gs_usb: USB suspended, leaving the bus");
            self.shared.command.signal(Command::Reset);
        }
    }

    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        if !is_gs_usb(&req) {
            return None;
        }
        Some(match GsRequest::from_u8(req.request) {
            Some(GsRequest::HostFormat) => OutResponse::Accepted,
            Some(GsRequest::Bittiming) if req.value == 0 => {
                let Ok(bt) = DeviceBittiming::decode(data) else { return Some(OutResponse::Rejected) };
                if let Some(timing) = C::NOMINAL.timing(&bt) {
                    info!("gs_usb: bit timing {}: {} bit/s", timing, bitrate::<C>(timing));
                    self.shared.timing.lock(|t| *t.borrow_mut() = Some(timing));
                    OutResponse::Accepted
                } else {
                    warn!("gs_usb: bit timing out of range");
                    OutResponse::Rejected
                }
            }
            Some(GsRequest::Mode) if req.value == 0 => match DeviceMode::decode(data) {
                Ok(DeviceMode { mode: mode::START, flags }) => self.start(flags),
                Ok(DeviceMode { mode: mode::RESET, .. }) => {
                    self.shared.command.signal(Command::Reset);
                    OutResponse::Accepted
                }
                _ => OutResponse::Rejected,
            },
            Some(GsRequest::Identify) if req.value == 0 => match decode_u32(data) {
                Ok(mode) => {
                    let on = mode == identify::ON;
                    info!("gs_usb: identify {}", on);
                    self.shared.identify.store(on, Ordering::Relaxed);
                    OutResponse::Accepted
                }
                Err(_) => OutResponse::Rejected,
            },
            _ => OutResponse::Rejected,
        })
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if !is_gs_usb(&req) {
            return None;
        }
        Some(match GsRequest::from_u8(req.request) {
            Some(GsRequest::BtConst) if req.value == 0 => InResponse::Accepted(
                BtConst {
                    feature: FEATURES,
                    fclk_can: C::CLOCK_HZ,
                    tseg1_min: 1,
                    tseg1_max: u32::from(C::NOMINAL.tseg1_max),
                    tseg2_min: 1,
                    tseg2_max: u32::from(C::NOMINAL.tseg2_max),
                    sjw_max: u32::from(C::NOMINAL.sjw_max),
                    brp_min: 1,
                    brp_max: u32::from(C::NOMINAL.brp_max),
                    brp_inc: 1,
                }
                .encode(buf),
            ),
            Some(GsRequest::DeviceConfig) => InResponse::Accepted(
                DeviceConfig {
                    icount: 0,
                    sw_version: SW_VERSION,
                    hw_version: HW_VERSION,
                }
                .encode(buf),
            ),
            Some(GsRequest::Timestamp) => {
                buf[..4].copy_from_slice(&micros(Instant::now()).to_le_bytes());
                InResponse::Accepted(&buf[..4])
            }
            Some(GsRequest::GetState) if req.value == 0 => InResponse::Accepted(self.state().encode(buf)),
            _ => InResponse::Rejected,
        })
    }
}

/// The bit rate `timing` gives from the channel's clock.
fn bitrate<C: Channel>(timing: Timing) -> u32 {
    let quanta = u32::from(timing.prescaler) * (1 + u32::from(timing.seg1) + u32::from(timing.seg2));
    C::CLOCK_HZ / quanta
}

/// The device's microsecond timer, as `gs_usb` stamps frames: it wraps.
#[expect(clippy::cast_possible_truncation, reason = "the timestamp is 32 bits and wraps; the host follows it")]
fn micros(instant: Instant) -> u32 {
    instant.as_micros() as u32
}

/// How long the channel stays bus-off before it starts recovering by itself,
/// as candleLight does: a node with a fault doesn't hammer the bus, and a
/// host that wants to act has time to.
const BUS_OFF_RESTART_DELAY: Duration = Duration::from_millis(100);

/// How often to look whether recovery has finished: bxCAN has no interrupt
/// for leaving bus-off. Recovery takes 128 times 11 recessive bits, 2.8 ms
/// on an idle bus at 500 kbit/s.
const RECOVERY_POLL: Duration = Duration::from_millis(10);

/// Where an automatic recovery from bus-off stands.
#[derive(Clone, Copy)]
enum Recovery {
    /// Not bus-off, or not recovering.
    Idle,
    /// Bus-off: recovery starts at this time.
    Due(Instant),
    /// Recovering: look again at this time.
    Waiting(Instant),
}

/// A frame the host sent, awaiting its report.
struct Pending {
    id: TransmitId,
    frame: HostFrame,
}

/// Moves frames between the bulk endpoints and the CAN channel, and acts on
/// the host's start and reset. Never returns.
///
/// On going bus-off the channel recovers by itself after
/// [`BUS_OFF_RESTART_DELAY`], and the host is told it restarted, as
/// candleLight and Kvaser's adapters do; Linux's `gs_usb` has no restart of
/// its own (`restart-ms` doesn't apply).
pub async fn bridge<C: Channel>(can: &mut C, ep_in: &mut impl EndpointIn, ep_out: &mut impl EndpointOut, shared: &Shared) -> ! {
    let mut pending: [Option<Pending>; 3] = [const { None }; 3];
    let mut running = false;
    let mut timestamps = false;
    // The bus state last reported to the host, and the frames lost so far.
    let mut bus = BusState::ErrorActive;
    let mut lost = 0;
    let mut recovery = Recovery::Idle;
    let mut out_buf = [0u8; 64];
    let mut in_buf = [0u8; CLASSIC_TS_LEN];

    loop {
        let room = running && pending.iter().any(Option::is_none);
        let read = async { if room { ep_out.read(&mut out_buf).await } else { never::<Result<usize, EndpointError>>().await } };
        let wake = match recovery {
            Recovery::Idle => None,
            Recovery::Due(at) | Recovery::Waiting(at) => Some(at),
        };
        let timer = async move {
            match wake {
                Some(at) => Timer::at(at).await,
                None => never::<()>().await,
            }
        };
        // Bus errors come only when the host asked for them (BERR_REPORTING).
        let events = select(can.next_event(), can.next_bus_error());
        match select4(read, events, shared.command.wait(), timer).await {
            Either4::First(Ok(len)) => from_host(can, ep_in, &mut in_buf, &mut pending, &out_buf[..len], timestamps).await,
            Either4::First(Err(EndpointError::Disabled)) => {
                info!("gs_usb: USB disabled, leaving the bus");
                stop(can, &mut pending, &mut running, shared);
                recovery = Recovery::Idle;
                ep_out.wait_enabled().await;
            }
            Either4::First(Err(EndpointError::BufferOverflow)) => warn!("gs_usb: OUT transfer too long"),
            Either4::Second(Either::First(event)) => match event {
                Event::Received(received) if running => {
                    let Some(mut host) = from_frame(received.frame(), ECHO_ID_RX) else {
                        warn!("gs_usb: an FD frame, not yet passed on");
                        continue;
                    };
                    host.timestamp_us = timestamps.then(|| micros(*received.timestamp()));
                    // Frames lost since the last one received: the host
                    // turns the flag into an overflow error frame.
                    let now_lost = can.lost_frames();
                    if now_lost > lost {
                        host.flags |= frame_flag::OVERFLOW;
                        lost = now_lost;
                    }
                    shared.rx_activity.store(true, Ordering::Relaxed);
                    write(ep_in, host.encode(&mut in_buf)).await;
                }
                Event::Received(_) => {}
                Event::Transmitted { id, outcome } => echo(ep_in, &mut in_buf, &mut pending, id, outcome, timestamps, shared).await,
                Event::BusStateChanged(state) => {
                    let counters = can.error_counters();
                    info!("gs_usb: bus {}, counters {}/{}", defmt::Debug2Format(&state), counters.transmit, counters.receive);
                    if running && state != bus {
                        report_state(ep_in, &mut in_buf, bus, state, counters, timestamps).await;
                    }
                    bus = state;
                    recovery = match state {
                        BusState::BusOff if running => Recovery::Due(Instant::now() + BUS_OFF_RESTART_DELAY),
                        BusState::BusOff => recovery,
                        _ => Recovery::Idle,
                    };
                }
            },
            Either4::Second(Either::Second(error)) => {
                if running {
                    let counters = can.error_counters();
                    let mut host = bus_error_frame(error, counters);
                    host.timestamp_us = timestamps.then(|| micros(Instant::now()));
                    write(ep_in, host.encode(&mut in_buf)).await;
                }
            }
            Either4::Third(Command::Start { settings, timestamps: stamped }) => {
                stop(can, &mut pending, &mut running, shared);
                recovery = Recovery::Idle;
                match can.start(&settings) {
                    Ok(()) => {
                        running = true;
                        shared.running.store(true, Ordering::Relaxed);
                        timestamps = stamped;
                        bus = BusState::ErrorActive;
                        lost = can.lost_frames();
                        info!("gs_usb: started, {}, timestamps {}", settings.mode, timestamps);
                    }
                    Err(e) => warn!("gs_usb: can't start the controller: {}", e),
                }
            }
            Either4::Third(Command::Reset) => {
                stop(can, &mut pending, &mut running, shared);
                recovery = Recovery::Idle;
                info!("gs_usb: reset");
            }
            Either4::Fourth(()) => {
                recovery = recover(can, ep_in, &mut in_buf, recovery, timestamps).await;
                if matches!(recovery, Recovery::Idle) {
                    bus = BusState::ErrorActive;
                }
            }
        }
    }
}

/// Hands a frame from the host to the channel, or, if this device can't send
/// it, echoes it unsent so the host's slot is freed.
async fn from_host<C: Channel>(can: &mut C, ep_in: &mut impl EndpointIn, in_buf: &mut [u8; CLASSIC_TS_LEN], pending: &mut [Option<Pending>; 3], bytes: &[u8], timestamps: bool) {
    let Ok(host) = HostFrame::decode(bytes) else {
        warn!("gs_usb: undecodable frame of {} bytes", bytes.len());
        return;
    };
    let sent = match to_frame(&host) {
        Some(frame) => can.transmit(&frame),
        None => Err(Fault::FdUnsupported),
    };
    match sent {
        Ok(id) => {
            if let Some(slot) = pending.iter_mut().find(|p| p.is_none()) {
                *slot = Some(Pending { id, frame: host });
            }
        }
        Err(e) => {
            warn!("gs_usb: frame not sent ({}), echoed unsent", e);
            let mut echo = host;
            echo.timestamp_us = timestamps.then(|| micros(Instant::now()));
            write(ep_in, echo.encode(in_buf)).await;
        }
    }
}

/// The next step of an automatic recovery from bus-off, once its time has
/// come: start it, or look whether it has finished and, if so, tell the host
/// the channel restarted. [`Recovery::Idle`] once it has.
async fn recover<C: Channel>(can: &mut C, ep_in: &mut impl EndpointIn, in_buf: &mut [u8; CLASSIC_TS_LEN], recovery: Recovery, timestamps: bool) -> Recovery {
    match recovery {
        Recovery::Due(_) => {
            info!("gs_usb: recovering from bus-off");
            if let Err(e) = can.recover() {
                warn!("gs_usb: can't start recovery: {}", e);
            }
            Recovery::Waiting(Instant::now() + RECOVERY_POLL)
        }
        Recovery::Waiting(_) if can.bus_state() != BusState::BusOff => {
            let counters = can.error_counters();
            info!("gs_usb: recovered, counters {}/{}", counters.transmit, counters.receive);
            report_state(ep_in, in_buf, BusState::BusOff, BusState::ErrorActive, counters, timestamps).await;
            Recovery::Idle
        }
        Recovery::Waiting(_) => Recovery::Waiting(Instant::now() + RECOVERY_POLL),
        Recovery::Idle => Recovery::Idle,
    }
}

/// Echoes the host's frame `id` now that the channel has reported it: sent,
/// or not (the echo frees the host's slot either way).
async fn echo(ep_in: &mut impl EndpointIn, in_buf: &mut [u8; CLASSIC_TS_LEN], pending: &mut [Option<Pending>; 3], id: TransmitId, outcome: Outcome<Instant>, timestamps: bool, shared: &Shared) {
    let Some(slot) = pending.iter_mut().find(|p| p.as_ref().is_some_and(|p| p.id == id)) else { return };
    let Some(Pending { frame: mut echo, .. }) = slot.take() else { return };
    let at = match outcome {
        Outcome::Sent(at) => {
            shared.tx_activity.store(true, Ordering::Relaxed);
            at
        }
        other => {
            debug!("gs_usb: frame {} not sent: {}", echo.echo_id, defmt::Debug2Format(&other));
            Instant::now()
        }
    };
    echo.timestamp_us = timestamps.then(|| micros(at));
    write(ep_in, echo.encode(in_buf)).await;
}

/// Tells the host the bus state went from `from` to `to`.
async fn report_state(ep_in: &mut impl EndpointIn, in_buf: &mut [u8; CLASSIC_TS_LEN], from: BusState, to: BusState, counters: ErrorCounters, timestamps: bool) {
    let mut host = error_frame(from, to, counters);
    host.timestamp_us = timestamps.then(|| micros(Instant::now()));
    write(ep_in, host.encode(in_buf)).await;
}

/// The error frame reporting a bus error, as candleLight writes it: a
/// protocol violation of its kind (or no acknowledge), with the counters.
fn bus_error_frame(error: BusError, counters: ErrorCounters) -> HostFrame {
    let mut can_id = can_id::ERR | can_err::PROT | can_err::BUSERROR | can_err::CNT;
    let mut data = [0; 8];
    match error {
        BusError::Stuff => data[2] = can_err::prot::STUFF,
        BusError::Form => data[2] = can_err::prot::FORM,
        BusError::Acknowledgement => can_id |= can_err::ACK,
        BusError::BitRecessive => data[2] = can_err::prot::BIT1,
        BusError::BitDominant => data[2] = can_err::prot::BIT0,
        BusError::Crc => data[3] = can_err::prot::LOC_CRC_SEQ,
        BusError::Other => {}
    }
    data[6] = counters.transmit;
    data[7] = counters.receive;
    HostFrame {
        echo_id: ECHO_ID_RX,
        can_id,
        can_dlc: can_err::DLC,
        channel: 0,
        flags: 0,
        data,
        timestamp_us: None,
    }
}

/// The error frame telling the host the bus state went from `from` to `to`,
/// as Linux's `gs_usb` reads it: bus-off and restarts in the identifier,
/// error passive and back to active in `data[1]`, and the error counters in
/// `data[6]` and `data[7]`.
fn error_frame(from: BusState, to: BusState, counters: ErrorCounters) -> HostFrame {
    let mut can_id = can_id::ERR | can_err::CNT;
    let mut data = [0; 8];
    data[6] = counters.transmit;
    data[7] = counters.receive;
    match to {
        BusState::BusOff => can_id |= can_err::BUSOFF,
        BusState::ErrorPassive => {
            can_id |= can_err::CRTL;
            let passive = |count: u8, flag: u8| if count >= 128 { flag } else { 0 };
            data[1] = passive(counters.transmit, can_err::crtl::TX_PASSIVE) | passive(counters.receive, can_err::crtl::RX_PASSIVE);
            if data[1] == 0 {
                // The counters fell back between the change and reading them.
                data[1] = can_err::crtl::TX_PASSIVE;
            }
        }
        BusState::ErrorActive if from == BusState::BusOff => can_id |= can_err::RESTARTED,
        BusState::ErrorActive => {
            can_id |= can_err::CRTL;
            data[1] = can_err::crtl::ACTIVE;
        }
    }
    HostFrame {
        echo_id: ECHO_ID_RX,
        can_id,
        can_dlc: can_err::DLC,
        channel: 0,
        flags: 0,
        data,
        timestamp_us: None,
    }
}

/// Leaves the bus and forgets the frames awaiting reports: the host has
/// forgotten them too.
fn stop<C: Channel>(can: &mut C, pending: &mut [Option<Pending>; 3], running: &mut bool, shared: &Shared) {
    if *running && let Err(e) = can.stop() {
        warn!("gs_usb: can't stop the controller: {}", e);
    }
    *running = false;
    shared.running.store(false, Ordering::Relaxed);
    *pending = [const { None }; 3];
}

async fn write(ep_in: &mut impl EndpointIn, bytes: &[u8]) {
    if let Err(e) = ep_in.write(bytes).await {
        warn!("gs_usb: IN transfer failed: {}", defmt::Debug2Format(&e));
    }
}

/// The CAN frame the host sent, if this device can send it: classic, not an
/// error frame.
fn to_frame(host: &HostFrame) -> Option<Frame> {
    if host.can_id & can_id::ERR != 0 {
        return None;
    }
    let id = if host.can_id & can_id::EFF != 0 {
        CanId::new_extended(host.can_id & can_id::EFF_MASK)?
    } else {
        CanId::new_standard(u16::try_from(host.can_id & can_id::SFF_MASK).ok()?)?
    };
    let dlc = usize::from(host.can_dlc);
    let frame = if host.can_id & can_id::RTR != 0 {
        CanFrame::new_remote(id, dlc)
    } else {
        CanFrame::new_with_dlc(id, &host.data[..dlc.min(8)], dlc)
    };
    frame.map(Frame::Can)
}

/// The host's form of `frame`, if it is classic.
fn from_frame(frame: &Frame, echo_id: u32) -> Option<HostFrame> {
    let Frame::Can(frame) = frame else { return None };
    let mut can_id = match frame.id() {
        CanId::Standard(id) => u32::from(id),
        CanId::Extended(id) => id | can_id::EFF,
    };
    if frame.is_remote_frame() {
        can_id |= can_id::RTR;
    }
    let mut data = [0; 8];
    data[..frame.data().len()].copy_from_slice(frame.data());
    Some(HostFrame {
        echo_id,
        can_id,
        #[expect(clippy::cast_possible_truncation, reason = "a DLC is at most 15")]
        can_dlc: frame.dlc() as u8,
        channel: 0,
        flags: 0,
        data,
        timestamp_us: None,
    })
}
