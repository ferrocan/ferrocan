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

use can_hal::{BusState, BusStatus, CanFrame, CanId, ConfirmedTransmit, ErrorCounters, Event, Outcome, Recover, Statistics, TransmitId};
use can_hal_bxcan::{BitTiming, Config, Mode};
use defmt::{debug, info, warn};
use embassy_futures::select::{Either4, select4};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::Handler;
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{EndpointError, EndpointIn, EndpointOut};
use gs_usb_protocol::{BtConst, CLASSIC_TS_LEN, DeviceBittiming, DeviceConfig, DeviceMode, ECHO_ID_RX, HostFrame, Request as GsRequest, can_err, can_id, feature, frame_flag, mode};

use crate::board::{CAN_CLOCK_HZ, CanChannel};

/// The features the device offers.
pub const FEATURES: u32 = feature::LISTEN_ONLY | feature::LOOP_BACK | feature::ONE_SHOT | feature::HW_TIMESTAMP;

/// The firmware's version, as `DEVICE_CONFIG` reports it.
const SW_VERSION: u32 = 1;
/// The hardware's version.
const HW_VERSION: u32 = 1;

/// What the host asked the channel to do.
pub enum Command {
    /// Join the bus with this configuration, stamping frames or not.
    Start { config: Config, timestamps: bool },
    /// Leave the bus.
    Reset,
}

/// What the control handler and the bridge share.
pub struct Shared {
    timing: Mutex<CriticalSectionRawMutex, RefCell<Option<BitTiming>>>,
    command: Signal<CriticalSectionRawMutex, Command>,
}

impl Shared {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            timing: Mutex::new(RefCell::new(None)),
            command: Signal::new(),
        }
    }
}

impl Default for Shared {
    fn default() -> Self {
        Self::new()
    }
}

/// The one channel's shared state.
pub static SHARED: Shared = Shared::new();

/// Answers the `gs_usb` vendor requests to interface 0.
pub struct Control {
    shared: &'static Shared,
}

impl Control {
    #[must_use]
    pub const fn new(shared: &'static Shared) -> Self {
        Self { shared }
    }

    fn start(&self, flags: u32) -> OutResponse {
        if flags & !FEATURES != 0 {
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
        let config = Config::new(timing).with_mode(mode).with_retransmit(flags & feature::ONE_SHOT == 0);
        self.shared.command.signal(Command::Start {
            config,
            timestamps: flags & feature::HW_TIMESTAMP != 0,
        });
        OutResponse::Accepted
    }
}

/// Whether `req` is for this device: a vendor request to interface 0.
fn is_gs_usb(req: &Request) -> bool {
    req.request_type == RequestType::Vendor && req.recipient == Recipient::Interface && req.index == 0
}

impl Handler for Control {
    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        if !is_gs_usb(&req) {
            return None;
        }
        Some(match GsRequest::from_u8(req.request) {
            Some(GsRequest::HostFormat) => OutResponse::Accepted,
            Some(GsRequest::Bittiming) if req.value == 0 => {
                let Ok(bt) = DeviceBittiming::decode(data) else { return Some(OutResponse::Rejected) };
                let timing = (|| {
                    let seg1 = u8::try_from(bt.prop_seg.checked_add(bt.phase_seg1)?).ok()?;
                    BitTiming::new(u16::try_from(bt.brp).ok()?, seg1, u8::try_from(bt.phase_seg2).ok()?, u8::try_from(bt.sjw).ok()?).ok()
                })();
                if let Some(timing) = timing {
                    info!("gs_usb: bit timing {}: {} bit/s", timing, timing.bitrate(CAN_CLOCK_HZ));
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
                    fclk_can: CAN_CLOCK_HZ,
                    tseg1_min: 1,
                    tseg1_max: 16,
                    tseg2_min: 1,
                    tseg2_max: 8,
                    sjw_max: 4,
                    brp_min: 1,
                    brp_max: 1024,
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
            _ => InResponse::Rejected,
        })
    }
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
pub async fn bridge(can: &mut CanChannel, ep_in: &mut impl EndpointIn, ep_out: &mut impl EndpointOut, shared: &Shared) -> ! {
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
        match select4(read, can.next_event(), shared.command.wait(), timer).await {
            Either4::First(Ok(len)) => from_host(can, ep_in, &mut in_buf, &mut pending, &out_buf[..len], timestamps).await,
            Either4::First(Err(EndpointError::Disabled)) => {
                info!("gs_usb: USB disabled, leaving the bus");
                stop(can, &mut pending, &mut running);
                recovery = Recovery::Idle;
                ep_out.wait_enabled().await;
            }
            Either4::First(Err(EndpointError::BufferOverflow)) => warn!("gs_usb: OUT transfer too long"),
            Either4::Second(event) => match event {
                Event::Received(received) if running => {
                    let mut host = from_can(received.frame(), ECHO_ID_RX);
                    host.timestamp_us = timestamps.then(|| micros(*received.timestamp()));
                    // Frames lost since the last one received: the host
                    // turns the flag into an overflow error frame.
                    let now_lost = lost_frames(can);
                    if now_lost > lost {
                        host.flags |= frame_flag::OVERFLOW;
                        lost = now_lost;
                    }
                    write(ep_in, host.encode(&mut in_buf)).await;
                }
                Event::Received(_) => {}
                Event::Transmitted { id, outcome } => {
                    let Some(slot) = pending.iter_mut().find(|p| p.as_ref().is_some_and(|p| p.id == id)) else { continue };
                    let Some(Pending { frame: mut echo, .. }) = slot.take() else { continue };
                    let at = match outcome {
                        Outcome::Sent(at) => at,
                        other => {
                            debug!("gs_usb: frame {} not sent: {}", echo.echo_id, defmt::Debug2Format(&other));
                            Instant::now()
                        }
                    };
                    echo.timestamp_us = timestamps.then(|| micros(at));
                    write(ep_in, echo.encode(&mut in_buf)).await;
                }
                Event::BusStateChanged(state) => {
                    let counters = can.error_counters().unwrap_or(ErrorCounters { transmit: 0, receive: 0 });
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
            Either4::Third(Command::Start { config, timestamps: stamped }) => {
                stop(can, &mut pending, &mut running);
                recovery = Recovery::Idle;
                match can.reconfigure(config) {
                    Ok(()) => {
                        can.start();
                        running = true;
                        timestamps = stamped;
                        bus = BusState::ErrorActive;
                        lost = lost_frames(can);
                        info!("gs_usb: started, {}, timestamps {}", config.mode, timestamps);
                    }
                    Err(e) => warn!("gs_usb: can't configure the controller: {}", e),
                }
            }
            Either4::Third(Command::Reset) => {
                stop(can, &mut pending, &mut running);
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
async fn from_host(can: &mut CanChannel, ep_in: &mut impl EndpointIn, in_buf: &mut [u8; CLASSIC_TS_LEN], pending: &mut [Option<Pending>; 3], bytes: &[u8], timestamps: bool) {
    let Ok(host) = HostFrame::decode(bytes) else {
        warn!("gs_usb: undecodable frame of {} bytes", bytes.len());
        return;
    };
    let Some(frame) = to_can(&host) else {
        warn!("gs_usb: unsupported frame, echoed unsent");
        let mut echo = host;
        echo.timestamp_us = timestamps.then(|| micros(Instant::now()));
        write(ep_in, echo.encode(in_buf)).await;
        return;
    };
    match can.try_transmit(&frame) {
        Ok(id) => {
            if let Some(slot) = pending.iter_mut().find(|p| p.is_none()) {
                *slot = Some(Pending { id, frame: host });
            }
        }
        Err(e) => warn!("gs_usb: no room for a frame: {}", defmt::Debug2Format(&e)),
    }
}

/// The next step of an automatic recovery from bus-off, once its time has
/// come: start it, or look whether it has finished and, if so, tell the host
/// the channel restarted. [`Recovery::Idle`] once it has.
async fn recover(can: &mut CanChannel, ep_in: &mut impl EndpointIn, in_buf: &mut [u8; CLASSIC_TS_LEN], recovery: Recovery, timestamps: bool) -> Recovery {
    match recovery {
        Recovery::Due(_) => {
            info!("gs_usb: recovering from bus-off");
            if let Err(e) = can.recover() {
                warn!("gs_usb: can't start recovery: {}", e);
            }
            Recovery::Waiting(Instant::now() + RECOVERY_POLL)
        }
        Recovery::Waiting(_) if can.bus_state().is_ok_and(|state| state != BusState::BusOff) => {
            let counters = can.error_counters().unwrap_or(ErrorCounters { transmit: 0, receive: 0 });
            info!("gs_usb: recovered, counters {}/{}", counters.transmit, counters.receive);
            report_state(ep_in, in_buf, BusState::BusOff, BusState::ErrorActive, counters, timestamps).await;
            Recovery::Idle
        }
        Recovery::Waiting(_) => Recovery::Waiting(Instant::now() + RECOVERY_POLL),
        Recovery::Idle => Recovery::Idle,
    }
}

/// Tells the host the bus state went from `from` to `to`.
async fn report_state(ep_in: &mut impl EndpointIn, in_buf: &mut [u8; CLASSIC_TS_LEN], from: BusState, to: BusState, counters: ErrorCounters, timestamps: bool) {
    let mut host = error_frame(from, to, counters);
    host.timestamp_us = timestamps.then(|| micros(Instant::now()));
    write(ep_in, host.encode(in_buf)).await;
}

/// The frames the channel has lost so far: dropped from its event queue, or
/// overrun in the controller's receive FIFOs.
fn lost_frames(can: &CanChannel) -> u64 {
    can.statistics().map_or(0, |s| s.rx_dropped.unwrap_or(0) + s.rx_overruns.unwrap_or(0))
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
fn stop(can: &mut CanChannel, pending: &mut [Option<Pending>; 3], running: &mut bool) {
    if *running && let Err(e) = can.stop() {
        warn!("gs_usb: can't stop the controller: {}", e);
    }
    *running = false;
    *pending = [const { None }; 3];
}

async fn write(ep_in: &mut impl EndpointIn, bytes: &[u8]) {
    if let Err(e) = ep_in.write(bytes).await {
        warn!("gs_usb: IN transfer failed: {}", defmt::Debug2Format(&e));
    }
}

/// The CAN frame the host sent, if this device can send it: classic, not an
/// error frame.
fn to_can(host: &HostFrame) -> Option<CanFrame> {
    if host.can_id & can_id::ERR != 0 {
        return None;
    }
    let id = if host.can_id & can_id::EFF != 0 {
        CanId::new_extended(host.can_id & can_id::EFF_MASK)?
    } else {
        CanId::new_standard(u16::try_from(host.can_id & can_id::SFF_MASK).ok()?)?
    };
    let dlc = usize::from(host.can_dlc);
    if host.can_id & can_id::RTR != 0 {
        CanFrame::new_remote(id, dlc)
    } else {
        CanFrame::new_with_dlc(id, &host.data[..dlc.min(8)], dlc)
    }
}

/// The host's form of `frame`.
fn from_can(frame: &CanFrame, echo_id: u32) -> HostFrame {
    let mut can_id = match frame.id() {
        CanId::Standard(id) => u32::from(id),
        CanId::Extended(id) => id | can_id::EFF,
    };
    if frame.is_remote_frame() {
        can_id |= can_id::RTR;
    }
    let mut data = [0; 8];
    data[..frame.data().len()].copy_from_slice(frame.data());
    HostFrame {
        echo_id,
        can_id,
        #[expect(clippy::cast_possible_truncation, reason = "a DLC is at most 15")]
        can_dlc: frame.dlc() as u8,
        channel: 0,
        flags: 0,
        data,
        timestamp_us: None,
    }
}
