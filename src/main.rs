#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

use panic_rtt_core::{self, rprint, rprintln, rtt_init_print};
use stm32f303cct6_robotdyn_bsp as bsp;
use stm32f3xx_hal as p_hal;

use cortex_m_rt::{entry, exception, ExceptionFrame};

use p_hal::hal::digital::v2::ToggleableOutputPin;

use freertos_rust::{Duration, FreeRtosAllocator, FreeRtosUtils, Queue, Task, TaskDelay};

/// a simple message type
type LemonMsg = [u8; 4];

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 64_000_000;

use core::sync::atomic::{AtomicPtr, AtomicUsize, Ordering};

use p_hal::prelude::*;
// use core::ptr::{null, null_mut};

use bsp::peripherals::{self, UserLed1Type};
use cortex_m::asm;

static GLOBAL_QUEUE_HANDLE: AtomicPtr<Queue<LemonMsg>> = AtomicPtr::new(core::ptr::null_mut());
static UPDATE_COUNT: AtomicUsize = AtomicUsize::new(0);
static USER_LED1: AtomicPtr<UserLed1Type> = AtomicPtr::new(core::ptr::null_mut());

/// cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
    asm::bkpt();
    loop {}
}

/// Called when FreeRTOS assert fails
#[no_mangle]
extern "C" fn handle_assert_failed() {
    asm::bkpt();
}

/// Called when FreeRTOS detects a stack overflow
#[no_mangle]
extern "C" fn vApplicationStackOverflowHook() {
    asm::bkpt();
}

#[alloc_error_handler]
fn alloc_error(_layout: core::alloc::Layout) -> ! {
    asm::bkpt();
    loop {}
}

/// Toggle the user leds from their prior state
fn toggle_leds() {
    unsafe {
        USER_LED1
            .load(Ordering::Relaxed)
            .as_mut()
            .unwrap()
            .toggle()
            .unwrap();
    }
}

/// RTOS calls this function to start Task 1
/// This task sends messages to the shared queue over and over
fn task1_start() {
    let mut send_val: u8 = 33;
    loop {
        let send_buf: LemonMsg = [send_val, 0, 0, 0];
        unsafe {
            GLOBAL_QUEUE_HANDLE
                .load(Ordering::Relaxed)
                .as_ref()
                .unwrap()
                .send(send_buf, Duration::ms(1000))
                .unwrap();
        }
        send_val = send_val.wrapping_add(1);
    }
}

/// RTOS calls this function to run Task 2
/// This task receives messages from queue over and over
fn task2_start() {
    let mut delayer = TaskDelay::new();

    loop {
        if let Ok(_msg) = unsafe {
            GLOBAL_QUEUE_HANDLE
                .load(Ordering::Relaxed)
                .as_ref()
                .unwrap()
                .receive(Duration::ms(20))
        } {

            toggle_leds();
            UPDATE_COUNT.fetch_add(1, Ordering::Relaxed);

            //this delay makes the LED blinking more perceptible
            delayer.delay_until(Duration::ms(50));
        }
    }
}

fn setup_tasks() {
    rprintln!("setup_tasks");

    Task::new()
        .name("task2")
        .stack_size(128)
        .start(task2_start)
        .unwrap();

    Task::new()
        .name("task1")
        .stack_size(128)
        .start(task1_start)
        .unwrap();

    rprintln!("setup_tasks done");
}

/// Setup peripherals
fn setup_peripherals() {
    rprint!("setup_peripherals...");

    let (mut user_led1, _delay_source, _i2c1_port, _spi1_port, _spi_csn, _usart1_port) =
        peripherals::setup_peripherals();

    //set initial states of user LEDs
    user_led1.set_high().unwrap();

    //store shared peripherals
    USER_LED1.store(&mut user_led1, Ordering::Relaxed);

    rprintln!("done!");
}

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- MAIN --");

    setup_peripherals();
    setup_tasks();

    let mut mq = Queue::new(10).unwrap();
    GLOBAL_QUEUE_HANDLE.store(&mut mq as *mut _, Ordering::Relaxed);

    //this should never return:
    FreeRtosUtils::start_scheduler()
}
