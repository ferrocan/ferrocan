// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The `gs_usb` device: the control requests, and the bridge between the bulk
//! endpoints and the CAN channel.
//!
//! The host's frames go to the channel as they come, while it has room; the
//! endpoint is not read otherwise, so USB holds the host back, one packet
//! late: the endpoint takes the next packet as soon as one is read, so it may
//! hold one the bridge hasn't asked for. That packet, and any transfer half
//! gathered, are discarded on reset and on start, so that nothing the host
//! sent before reaches the bus after. Each is echoed
//! to the host when the channel reports it, so an echo means the frame was
//! acknowledged on the bus, not merely queued. A frame aborted or given up is
//! echoed too, as Linux's `gs_usb` frees a transmit slot only on an echo; it
//! has no way to be told a frame failed. A host that asks for transmit
//! confirmation ([`feature::TX_CONFIRMATION`], ours: `can_usb_gs`) is told,
//! by [`frame_flag::TX_FAILED`] on the echo and the reason in its first
//! data byte.

use core::cell::RefCell;
use core::future::pending as never;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use can_hal::{Abort, BusState, BusStatus, CanFdFrame, CanFrame, CanId, ErrorCounters, Event, Failure, Frame, Outcome, TransmitId};
use defmt::{debug, info, warn};
use embassy_futures::poll_once;
use embassy_futures::select::{Either, Either3, Either4, select, select3, select4};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel as Queue;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::Handler;
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{EndpointError, EndpointIn, EndpointOut};
use gs_usb_protocol::{BtConst, BtConstExt, DeviceBittiming, DeviceConfig, DeviceMode, DeviceState, ECHO_ID_RX, FD_TS_LEN, HostFrame, Request as GsRequest, can_err, can_id, decode_u32, fd_len, feature, frame_flag, identify, mode, state, tx_failed};

use crate::channel::{BusError, Channel, FailureCause, Fault, Mode, Settings, Timing};

/// The features the device offers with a channel of type `C`.
const fn features<C: Channel>() -> u32 {
    let fd = if C::DATA.is_some() { feature::BT_CONST_EXT } else { 0 };
    mode_flags::<C>() | feature::IDENTIFY | feature::GET_STATE | fd
}

/// The flags the host may ask for when starting the channel: the features
/// that are modes, CAN FD among them if the controller does it.
const fn mode_flags<C: Channel>() -> u32 {
    let fd = if C::DATA.is_some() { feature::FD } else { 0 };
    feature::LISTEN_ONLY | feature::LOOP_BACK | feature::ONE_SHOT | feature::HW_TIMESTAMP | feature::BERR_REPORTING | feature::TX_CONFIRMATION | fd
}

/// Bulk packets: a classic frame, with its timestamp, is 24 bytes; an FD
/// frame 76 from the host and 80 to it, in two packets.
pub const MAX_PACKET: u16 = 64;
const PACKET: usize = MAX_PACKET as usize;

/// Room for a frame from the host: the longest Linux sends is 76 bytes, and
/// a packet is read whole.
const OUT_BUF: usize = 2 * PACKET;

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
    /// Join the bus with these settings, for a session as the host asked.
    Start { settings: Settings, session: Session },
    /// Leave the bus.
    Reset,
}

/// What the host asked of a session besides the channel's settings.
#[derive(Clone, Copy, Default)]
pub struct Session {
    /// Stamp the frames to the host.
    pub timestamps: bool,
    /// Say on the echo when a frame wasn't sent, and why.
    pub tx_confirmation: bool,
}

/// What the control handler and the bridge share.
pub struct Shared {
    timing: Mutex<CriticalSectionRawMutex, RefCell<Option<Timing>>>,
    /// The data phase's, for CAN FD.
    data_timing: Mutex<CriticalSectionRawMutex, RefCell<Option<Timing>>>,
    command: Signal<CriticalSectionRawMutex, Command>,
    /// The `echo_id`s of frames the host asked to withdraw.
    aborts: Queue<CriticalSectionRawMutex, u32, 4>,
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
            data_timing: Mutex::new(RefCell::new(None)),
            command: Signal::new(),
            aborts: Queue::new(),
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
        if flags & !mode_flags::<C>() != 0 {
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
        let data = if flags & feature::FD == 0 {
            None
        } else if let Some(data) = self.shared.data_timing.lock(|t| *t.borrow()) {
            Some(data)
        } else {
            warn!("gs_usb: FD start before data bit timing");
            return OutResponse::Rejected;
        };
        let settings = Settings {
            nominal: timing,
            data,
            mode,
            retransmit: flags & feature::ONE_SHOT == 0,
            bus_errors: flags & feature::BERR_REPORTING != 0,
        };
        let session = Session {
            timestamps: flags & feature::HW_TIMESTAMP != 0,
            tx_confirmation: flags & feature::TX_CONFIRMATION != 0,
        };
        self.shared.command.signal(Command::Start { settings, session });
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
            Some(GsRequest::DataBittiming) if req.value == 0 => {
                let Ok(bt) = DeviceBittiming::decode(data) else { return Some(OutResponse::Rejected) };
                if let Some(timing) = C::DATA.and_then(|limits| limits.timing(&bt)) {
                    info!("gs_usb: data bit timing {}: {} bit/s", timing, bitrate::<C>(timing));
                    self.shared.data_timing.lock(|t| *t.borrow_mut() = Some(timing));
                    OutResponse::Accepted
                } else {
                    warn!("gs_usb: data bit timing out of range");
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
            // Accepted whatever the session: the frame's echo tells.
            Some(GsRequest::Abort) if req.value == 0 => match decode_u32(data) {
                Ok(echo_id) => {
                    if self.shared.aborts.try_send(echo_id).is_err() {
                        warn!("gs_usb: too many aborts waiting, {} dropped", echo_id);
                    }
                    OutResponse::Accepted
                }
                Err(_) => OutResponse::Rejected,
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
            Some(GsRequest::BtConst) if req.value == 0 => InResponse::Accepted(bt_const::<C>().encode(buf)),
            Some(GsRequest::BtConstExt) if req.value == 0 => match C::DATA {
                Some(data) => InResponse::Accepted(
                    BtConstExt {
                        nominal: bt_const::<C>(),
                        dtseg1_min: 1,
                        dtseg1_max: u32::from(data.tseg1_max),
                        dtseg2_min: 1,
                        dtseg2_max: u32::from(data.tseg2_max),
                        dsjw_max: u32::from(data.sjw_max),
                        dbrp_min: 1,
                        dbrp_max: u32::from(data.brp_max),
                        dbrp_inc: 1,
                    }
                    .encode(buf),
                ),
                None => InResponse::Rejected,
            },
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

/// `BT_CONST`: the features, the clock and the nominal timing's limits.
fn bt_const<C: Channel>() -> BtConst {
    BtConst {
        feature: features::<C>(),
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

/// The error state as Linux's `gs_usb` tracks it: can-hal's [`BusState`]
/// with the warning level between active and passive.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, defmt::Format)]
enum Level {
    Active,
    Warning,
    Passive,
    BusOff,
}

impl Level {
    /// The level of a channel in `state`, with an error counter at the
    /// warning level or not.
    const fn of(state: BusState, warning: bool) -> Self {
        match state {
            BusState::BusOff => Self::BusOff,
            BusState::ErrorPassive => Self::Passive,
            BusState::ErrorActive if warning => Self::Warning,
            BusState::ErrorActive => Self::Active,
        }
    }

    /// The level one error counter puts the channel at (bus-off is not
    /// the counters' to say).
    const fn of_counter(count: u8) -> Self {
        if count >= 128 {
            Self::Passive
        } else if count >= WARNING_LEVEL {
            Self::Warning
        } else {
            Self::Active
        }
    }

    /// The `data[1]` flag for the transmit counter at this level.
    const fn tx_flag(self) -> u8 {
        match self {
            Self::Active => can_err::crtl::ACTIVE,
            Self::Warning => can_err::crtl::TX_WARNING,
            Self::Passive => can_err::crtl::TX_PASSIVE,
            Self::BusOff => 0,
        }
    }

    /// The `data[1]` flag for the receive counter at this level.
    const fn rx_flag(self) -> u8 {
        match self {
            Self::Active => can_err::crtl::ACTIVE,
            Self::Warning => can_err::crtl::RX_WARNING,
            Self::Passive => can_err::crtl::RX_PASSIVE,
            Self::BusOff => 0,
        }
    }
}

/// The error state as the channel last reported it, and the level last
/// reported to the host.
struct ErrorState {
    bus: BusState,
    warning: bool,
    reported: Level,
}

impl ErrorState {
    /// As from power-on, and after recovery from bus-off, which clears the
    /// error counters.
    const CLEAR: Self = Self {
        bus: BusState::ErrorActive,
        warning: false,
        reported: Level::Active,
    };

    /// The change of level to tell the host about, if there is one, taken as
    /// reported.
    fn change(&mut self) -> Option<(Level, Level)> {
        let now = Level::of(self.bus, self.warning);
        let from = core::mem::replace(&mut self.reported, now);
        (now != from).then_some((from, now))
    }
}

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

impl Recovery {
    /// Waits until the recovery has something to do; for ever when idle.
    async fn due(self) {
        match self {
            Self::Idle => never().await,
            Self::Due(at) | Self::Waiting(at) => Timer::at(at).await,
        }
    }
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
    let mut session = Session::default();
    // The error state, and the frames lost so far.
    let mut errors = ErrorState::CLEAR;
    let mut lost = 0;
    let mut recovery = Recovery::Idle;
    let mut out = OutTransfer::new();
    let mut in_buf = [0u8; FD_TS_LEN];

    loop {
        // No frame is taken from the host while a start or reset waits: it
        // may be one the host sent before, which the command discards. (The
        // bridge can be held up writing to a host that has gone, while a
        // new one has already asked for a reset.) Nor while an abort waits:
        // it is for a frame pending when it was asked, and the next may
        // reuse that frame's `echo_id`.
        let room = running && !shared.command.signaled() && shared.aborts.is_empty() && pending.iter().any(Option::is_none);
        let reading = room || out.is_partial();
        let packet = out.packet();
        let read = async { if reading { ep_out.read(packet).await } else { never::<Result<usize, EndpointError>>().await } };
        // Bus errors come only when the host asked for them (BERR_REPORTING).
        let events = select3(can.next_event(), can.next_bus_error(), can.next_warning_change());
        match select4(read, events, select(shared.command.wait(), shared.aborts.receive()), recovery.due()).await {
            Either4::First(Ok(len)) => {
                if let Some(bytes) = out.received(len) {
                    from_host(can, ep_in, &mut in_buf, &mut pending, bytes, session.timestamps).await;
                }
            }
            Either4::First(Err(EndpointError::Disabled)) => {
                info!("gs_usb: USB disabled, leaving the bus");
                stop(can, ep_out, &mut pending, &mut out, &mut running, shared);
                recovery = Recovery::Idle;
                ep_out.wait_enabled().await;
            }
            Either4::First(Err(EndpointError::BufferOverflow)) => warn!("gs_usb: OUT transfer too long"),
            Either4::Second(Either3::First(event)) => match event {
                Event::Received(received) if running => {
                    let mut host = from_frame(received.frame(), ECHO_ID_RX);
                    host.timestamp_us = session.timestamps.then(|| micros(*received.timestamp()));
                    host.flags |= overflow(can.lost_frames(), &mut lost);
                    shared.rx_activity.store(true, Ordering::Relaxed);
                    write(ep_in, host.encode(&mut in_buf)).await;
                }
                Event::Received(_) => {}
                Event::Transmitted { id, outcome } => {
                    if matches!(outcome, Outcome::Sent(_)) {
                        shared.tx_activity.store(true, Ordering::Relaxed);
                    }
                    let failed = if session.tx_confirmation { failure_reason(&outcome, errors.bus, can.failure_cause(id)) } else { None };
                    echo(ep_in, &mut in_buf, &mut pending, id, outcome, failed, session.timestamps).await;
                }
                Event::BusStateChanged(state) => {
                    let counters = can.error_counters();
                    info!("gs_usb: bus {}, counters {}/{}", defmt::Debug2Format(&state), counters.transmit, counters.receive);
                    errors.bus = state;
                    errors.warning = latest_warning(can, errors.warning);
                    report_change(ep_in, &mut in_buf, &mut errors, running, counters, session.timestamps).await;
                    recovery = match state {
                        BusState::BusOff if running => Recovery::Due(Instant::now() + BUS_OFF_RESTART_DELAY),
                        BusState::BusOff => recovery,
                        _ => Recovery::Idle,
                    };
                }
            },
            Either4::Second(Either3::Third(now)) => {
                let counters = can.error_counters();
                info!("gs_usb: warning level {}, counters {}/{}", now, counters.transmit, counters.receive);
                errors.warning = now;
                report_change(ep_in, &mut in_buf, &mut errors, running, counters, session.timestamps).await;
            }
            Either4::Second(Either3::Second(error)) if running => report_bus_error(ep_in, &mut in_buf, error, can.error_counters(), session.timestamps).await,
            Either4::Second(Either3::Second(_)) => {}
            Either4::Third(Either::Second(echo_id)) => abort(can, &pending, echo_id),
            Either4::Third(Either::First(Command::Start { settings, session: asked })) => {
                stop(can, ep_out, &mut pending, &mut out, &mut running, shared);
                recovery = Recovery::Idle;
                match can.start(&settings) {
                    Ok(()) => {
                        running = true;
                        shared.running.store(true, Ordering::Relaxed);
                        session = asked;
                        errors = ErrorState::CLEAR;
                        lost = can.lost_frames();
                        info!("gs_usb: started, {}, timestamps {}, TX confirmation {}", settings.mode, session.timestamps, session.tx_confirmation);
                    }
                    Err(e) => warn!("gs_usb: can't start the controller: {}", e),
                }
            }
            Either4::Third(Either::First(Command::Reset)) => {
                stop(can, ep_out, &mut pending, &mut out, &mut running, shared);
                recovery = Recovery::Idle;
                info!("gs_usb: reset");
            }
            Either4::Fourth(()) => {
                recovery = recover(can, ep_in, &mut in_buf, recovery, session.timestamps).await;
                if matches!(recovery, Recovery::Idle) {
                    errors = ErrorState::CLEAR;
                }
            }
        }
    }
}

/// The flag that tells the host frames were lost since the last one received
/// (it turns it into an overflow error frame), if `now_lost`, the count so
/// far, has passed `lost`, which it updates.
fn overflow(now_lost: u64, lost: &mut u64) -> u8 {
    if now_lost > *lost {
        *lost = now_lost;
        frame_flag::OVERFLOW
    } else {
        0
    }
}

/// A frame from the host, which may take more than one packet: they gather
/// here, between turns of the bridge's loop, until a short one ends it. (A
/// packet is read whole or not at all, so a read the loop drops loses
/// nothing.)
struct OutTransfer {
    buf: [u8; OUT_BUF],
    len: usize,
    /// Dropping a transfer too long for the buffer, until it ends.
    discarding: bool,
}

impl OutTransfer {
    const fn new() -> Self {
        Self { buf: [0; OUT_BUF], len: 0, discarding: false }
    }

    /// Whether a transfer has begun and not ended.
    const fn is_partial(&self) -> bool {
        self.len > 0 || self.discarding
    }

    /// Where the next packet goes.
    fn packet(&mut self) -> &mut [u8] {
        &mut self.buf[self.len..][..PACKET]
    }

    /// A packet of `len` bytes was read into [`packet`](Self::packet): the
    /// whole transfer, if that ended it.
    fn received(&mut self, len: usize) -> Option<&[u8]> {
        self.len += len;
        if len == PACKET {
            if self.len + PACKET > OUT_BUF {
                warn!("gs_usb: OUT transfer too long, discarded");
                (self.len, self.discarding) = (0, true);
            }
            return None;
        }
        let end = core::mem::take(&mut self.len);
        if core::mem::take(&mut self.discarding) { None } else { Some(&self.buf[..end]) }
    }
}

/// Discards what the host sent before a reset or a start: the packet the
/// endpoint took without being asked (it is armed again as soon as one is
/// read), and any transfer half gathered. A read that finds a packet waiting
/// completes on its first poll; one that doesn't is dropped, which leaves the
/// endpoint as it was.
fn discard_out(ep_out: &mut impl EndpointOut, out: &mut OutTransfer) {
    *out = OutTransfer::new();
    let mut packet = [0u8; PACKET];
    while let Poll::Ready(Ok(len)) = poll_once(ep_out.read(&mut packet)) {
        debug!("gs_usb: discarded {} bytes the host sent before", len);
    }
}

/// Hands a frame from the host to the channel, or, if this device can't send
/// it, echoes it unsent so the host's slot is freed.
async fn from_host<C: Channel>(can: &mut C, ep_in: &mut impl EndpointIn, in_buf: &mut [u8; FD_TS_LEN], pending: &mut [Option<Pending>; 3], bytes: &[u8], timestamps: bool) {
    let Ok(host) = HostFrame::decode(bytes) else {
        warn!("gs_usb: undecodable frame of {} bytes", bytes.len());
        return;
    };
    debug!("gs_usb: frame from the host, echo id {}, id {=u32:#x}", host.echo_id, host.can_id);
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
async fn recover<C: Channel>(can: &mut C, ep_in: &mut impl EndpointIn, in_buf: &mut [u8; FD_TS_LEN], recovery: Recovery, timestamps: bool) -> Recovery {
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
            report_state(ep_in, in_buf, Level::BusOff, Level::Active, counters, timestamps).await;
            Recovery::Idle
        }
        Recovery::Waiting(_) => Recovery::Waiting(Instant::now() + RECOVERY_POLL),
        Recovery::Idle => Recovery::Idle,
    }
}

/// Why a frame wasn't sent, as its echo says it with transmit confirmation
/// on ([`tx_failed`]), the channel being in `bus` and the controller giving
/// `cause`; `None` if it was sent. Frames the channel gives up while bus-off
/// were dropped on going bus-off.
fn failure_reason(outcome: &Outcome<Instant>, bus: BusState, cause: Option<FailureCause>) -> Option<u8> {
    match outcome {
        Outcome::Sent(_) => None,
        Outcome::Aborted(Abort::Confirmed) => Some(tx_failed::ABORTED),
        Outcome::Aborted(_) => Some(tx_failed::ABORTED_UNCONFIRMED),
        Outcome::Failed(Failure::GaveUp) if bus == BusState::BusOff => Some(tx_failed::BUS_OFF),
        Outcome::Failed(Failure::GaveUp) => Some(match cause {
            Some(FailureCause::LostArbitration) => tx_failed::LOST_ARBITRATION,
            Some(FailureCause::Error) => tx_failed::TX_ERROR,
            None => tx_failed::GAVE_UP,
        }),
        // Unconfirmed, or a failure can-hal adds later: it may have been sent.
        Outcome::Failed(_) => Some(tx_failed::UNCONFIRMED),
    }
}

/// Echoes the host's frame `id` now that the channel has reported it: sent,
/// or not (the echo frees the host's slot either way). With `failed`, the
/// echo says it wasn't sent and why.
async fn echo(ep_in: &mut impl EndpointIn, in_buf: &mut [u8; FD_TS_LEN], pending: &mut [Option<Pending>; 3], id: TransmitId, outcome: Outcome<Instant>, failed: Option<u8>, timestamps: bool) {
    let Some(slot) = pending.iter_mut().find(|p| p.as_ref().is_some_and(|p| p.id == id)) else { return };
    let Some(Pending { frame: mut echo, .. }) = slot.take() else { return };
    let at = match outcome {
        Outcome::Sent(at) => at,
        other => {
            debug!("gs_usb: frame {} not sent: {}", echo.echo_id, defmt::Debug2Format(&other));
            Instant::now()
        }
    };
    if let Some(reason) = failed {
        echo.flags |= frame_flag::TX_FAILED;
        echo.data[0] = reason;
    }
    echo.timestamp_us = timestamps.then(|| micros(at));
    write(ep_in, echo.encode(in_buf)).await;
}

/// The channel's warning level as it now is, if it changed since last
/// taken, else `warning`: a change of bus state may come before the change
/// of warning level that goes with it (recovering from bus-off clears the
/// counters, say).
fn latest_warning<C: Channel>(can: &C, warning: bool) -> bool {
    match poll_once(can.next_warning_change()) {
        Poll::Ready(now) => now,
        Poll::Pending => warning,
    }
}

/// Tells the host, while `running`, of a change in `errors`' level. Leaving
/// bus-off is told as a restart, as candleLight does, then any change from
/// error active (the bus may already have raised the counters again).
async fn report_change(ep_in: &mut impl EndpointIn, in_buf: &mut [u8; FD_TS_LEN], errors: &mut ErrorState, running: bool, counters: ErrorCounters, timestamps: bool) {
    let Some((from, to)) = errors.change() else { return };
    if !running {
        return;
    }
    if from == Level::BusOff && to != Level::Active {
        report_state(ep_in, in_buf, from, Level::Active, counters, timestamps).await;
        report_state(ep_in, in_buf, Level::Active, to, counters, timestamps).await;
    } else {
        report_state(ep_in, in_buf, from, to, counters, timestamps).await;
    }
}

/// Tells the host of a bus error, with the counters after it.
async fn report_bus_error(ep_in: &mut impl EndpointIn, in_buf: &mut [u8; FD_TS_LEN], error: BusError, counters: ErrorCounters, timestamps: bool) {
    let mut host = bus_error_frame(error, counters);
    host.timestamp_us = timestamps.then(|| micros(Instant::now()));
    write(ep_in, host.encode(in_buf)).await;
}

/// Tells the host the level went from `from` to `to`.
async fn report_state(ep_in: &mut impl EndpointIn, in_buf: &mut [u8; FD_TS_LEN], from: Level, to: Level, counters: ErrorCounters, timestamps: bool) {
    let mut host = error_frame(from, to, counters);
    host.timestamp_us = timestamps.then(|| micros(Instant::now()));
    write(ep_in, host.encode(in_buf)).await;
}

/// The error frame reporting a bus error, as candleLight writes it: a
/// protocol violation of its kind (or no acknowledge), with the counters.
fn bus_error_frame(error: BusError, counters: ErrorCounters) -> HostFrame {
    let mut can_id = can_id::ERR | can_err::PROT | can_err::BUSERROR | can_err::CNT;
    let mut data = [0; 64];
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

/// The error frame telling the host the level went from `from` to `to`, as
/// Linux's `gs_usb` reads it: bus-off and restarts in the identifier, the
/// other levels in `data[1]`, and the error counters in `data[6]` and
/// `data[7]`.
fn error_frame(from: Level, to: Level, counters: ErrorCounters) -> HostFrame {
    let mut can_id = can_id::ERR | can_err::CNT;
    let mut data = [0; 64];
    data[6] = counters.transmit;
    data[7] = counters.receive;
    match to {
        Level::BusOff => can_id |= can_err::BUSOFF,
        Level::Active if from == Level::BusOff => can_id |= can_err::RESTARTED,
        _ => {
            can_id |= can_err::CRTL;
            data[1] = crtl_flags(to, counters);
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

/// The `data[1]` flags for `level`, as candleLight sets them: the flag of
/// whichever counter is at the higher level, or both when they are at the
/// same one (Linux takes warning before passive, so the lower must not come
/// with the higher). When the counters moved between the change and reading
/// them, the transmit counter's flag for `level`.
fn crtl_flags(level: Level, counters: ErrorCounters) -> u8 {
    let tx = Level::of_counter(counters.transmit);
    let rx = Level::of_counter(counters.receive);
    if tx.max(rx) != level {
        return level.tx_flag();
    }
    let mut flags = 0;
    if tx >= rx {
        flags |= tx.tx_flag();
    }
    if tx <= rx {
        flags |= rx.rx_flag();
    }
    flags
}

/// Leaves the bus and forgets everything the host sent before: the frames
/// awaiting reports, and what [`discard_out`] drops. The host has forgotten
/// them too.
fn stop<C: Channel>(can: &mut C, ep_out: &mut impl EndpointOut, pending: &mut [Option<Pending>; 3], out: &mut OutTransfer, running: &mut bool, shared: &Shared) {
    if *running && let Err(e) = can.stop() {
        warn!("gs_usb: can't stop the controller: {}", e);
    }
    *running = false;
    shared.running.store(false, Ordering::Relaxed);
    *pending = [const { None }; 3];
    discard_out(ep_out, out);
    // Aborts asked for before are for frames now forgotten, and their
    // `echo_id`s may come again.
    while shared.aborts.try_receive().is_ok() {}
}

/// Asks the channel to withdraw the host's frame `echo_id`, if it is pending
/// (one already reported is ignored); its echo says whether it was.
fn abort<C: Channel>(can: &mut C, pending: &[Option<Pending>; 3], echo_id: u32) {
    if let Some(frame) = pending.iter().flatten().find(|p| p.frame.echo_id == echo_id) {
        debug!("gs_usb: aborting frame {}", echo_id);
        if let Err(e) = can.abort(frame.id) {
            warn!("gs_usb: can't abort frame {}: {}", echo_id, e);
        }
    } else {
        debug!("gs_usb: abort of frame {}, not pending", echo_id);
    }
}

/// Sends `bytes` to the host as one transfer: full packets, then the rest,
/// or a zero-length packet if it ends on a packet's boundary.
async fn write(ep_in: &mut impl EndpointIn, bytes: &[u8]) {
    let mut packets = bytes.chunks(PACKET);
    let ends_full = bytes.len().is_multiple_of(PACKET);
    let zlp: &[u8] = &[];
    for packet in packets.by_ref().chain(ends_full.then_some(zlp)) {
        if let Err(e) = ep_in.write(packet).await {
            warn!("gs_usb: IN transfer failed: {}", defmt::Debug2Format(&e));
            return;
        }
    }
}

/// The CAN frame the host sent, if this device can send it: not an error
/// frame, nor an FD remote frame.
fn to_frame(host: &HostFrame) -> Option<Frame> {
    if host.can_id & can_id::ERR != 0 {
        return None;
    }
    let id = if host.can_id & can_id::EFF != 0 {
        CanId::new_extended(host.can_id & can_id::EFF_MASK)?
    } else {
        CanId::new_standard(u16::try_from(host.can_id & can_id::SFF_MASK).ok()?)?
    };
    if host.is_fd() {
        if host.can_id & can_id::RTR != 0 {
            return None;
        }
        let data = &host.data[..fd_len(host.can_dlc)];
        return CanFdFrame::new(id, data, host.flags & frame_flag::BRS != 0, host.flags & frame_flag::ESI != 0).map(Frame::Fd);
    }
    let dlc = usize::from(host.can_dlc);
    let frame = if host.can_id & can_id::RTR != 0 {
        CanFrame::new_remote(id, dlc)
    } else {
        CanFrame::new_with_dlc(id, &host.data[..dlc.min(8)], dlc)
    };
    frame.map(Frame::Can)
}

/// The host's form of `frame`.
fn from_frame(frame: &Frame, echo_id: u32) -> HostFrame {
    let mut can_id = match frame.id() {
        CanId::Standard(id) => u32::from(id),
        CanId::Extended(id) => id | can_id::EFF,
    };
    if frame.is_remote_frame() {
        can_id |= can_id::RTR;
    }
    let flags = match frame {
        Frame::Can(_) => 0,
        Frame::Fd(fd) => frame_flag::FD | if fd.brs() { frame_flag::BRS } else { 0 } | if fd.esi() { frame_flag::ESI } else { 0 },
    };
    let mut data = [0; 64];
    data[..frame.len()].copy_from_slice(frame.data());
    HostFrame {
        echo_id,
        can_id,
        #[expect(clippy::cast_possible_truncation, reason = "a DLC is at most 15")]
        can_dlc: frame.dlc() as u8,
        channel: 0,
        flags,
        data,
        timestamp_us: None,
    }
}
