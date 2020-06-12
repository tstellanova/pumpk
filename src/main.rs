#![no_main]
#![no_std]

#![feature(alloc_error_handler)]

use panic_rtt_core::{self, rprint, rprintln, rtt_init_print};
use stm32f3xx_hal as p_hal;
use stm32f303cct6_robotdyn_bsp as bsp;

use cortex_m_rt::{entry, exception, ExceptionFrame};

use p_hal::hal::digital::v2::ToggleableOutputPin;

use freertos_rust::{FreeRtosAllocator, FreeRtosUtils, Task, Queue, Duration, TaskDelay};


type LemonMsg =  [u8; 4];

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;


#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 64_000_000;


use core::sync::atomic::{AtomicUsize, Ordering, AtomicPtr};


use p_hal::{prelude::*};
// use core::ptr::{null, null_mut};

use bsp::peripherals::{self, UserLed1Type};
use cortex_m::asm;


static GLOBAL_QUEUE_HANDLE: AtomicPtr<Queue<LemonMsg>> = AtomicPtr::new(core::ptr::null_mut());
static UPDATE_COUNT: AtomicUsize = AtomicUsize::new(0);
static USER_LED1: AtomicPtr<UserLed1Type> =  AtomicPtr::new(core::ptr::null_mut());

// cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
    rprintln!("HardFault {:?}", _ef);
    loop {
    }
}

#[exception]
fn DefaultHandler(val: i16) -> ! {
    rprintln!("DefaultHandler {}", val);
    loop {
    }
}

/// Called when FreeRTOS assert fails
#[no_mangle]
extern "C" fn handle_assert_failed() {
    rprintln!("handle_assert_failed");
}

/// Toggle the user leds from their prior state
fn toggle_leds() {
    unsafe {
        USER_LED1.load(Ordering::Relaxed).as_mut().unwrap().toggle().unwrap();
    }
}

// main body of Task 1
fn task1_body(send_val: u8)  {
    let send_buf:[u8; 4] = [send_val, 0, 0, 0];
    unsafe {
        GLOBAL_QUEUE_HANDLE.load(Ordering::Relaxed).as_ref().unwrap()
            .send(send_buf, Duration::ms(250)).unwrap();
    }

}

/// RTOS calls this function to start Task 1
fn task1_start() {
    //rprintln!("task1_start");

    let mut send_val:u8 = 0;
    loop {
        let _ = task1_body(send_val);
        send_val = send_val.wrapping_add(1);
        //TODO yield?
    };
}


/// main body of Task 2
fn task2_body() -> i32 {
    if let Ok(_data) =
        unsafe {
            GLOBAL_QUEUE_HANDLE.load(Ordering::Relaxed).as_ref().unwrap()
                .receive(Duration::ms(50))
        }
    {
        toggle_leds();
        UPDATE_COUNT.fetch_add(1, Ordering::SeqCst);
    }

    0
}

/// RTOS calls this function to run Task 2
fn task2_start() {
    //rprintln!("task2_start");
    let mut delayer = TaskDelay::new();

    loop {
        let _ = task2_body();
        delayer.delay_until(Duration::ms(50));
        //this delay is not necessary, but it makes the LED blinking more perceptible
        //TODO
        //cmsis_rtos2::rtos_os_delay(50);
    };
}


pub fn setup_tasks() {
    rprint!("setup_tasks");

    let mut mq = Queue::new(10).unwrap();
    GLOBAL_QUEUE_HANDLE.store(&mut mq as *mut _, Ordering::Relaxed);

    Task::new().name("task1").stack_size(128).start(task1_start).unwrap();
    Task::new().name("task2").stack_size(128).start(task2_start).unwrap();

    rprintln!("...done");
}



#[alloc_error_handler]
fn alloc_error(_layout: core::alloc::Layout) -> ! {
    //set_led(true);
    asm::bkpt();
    loop {}
}

/// Setup peripherals
fn setup_peripherals()   {
    rprint!( "setup_peripherals...");

    let (mut user_led1,
        _delay_source,
        _i2c1_port,
        _spi1_port,
        _spi_csn,
        _usart1_port
    ) =
        peripherals::setup_peripherals();

    //set initial states of user LEDs
    user_led1.set_high().unwrap();

    //store shared peripherals
    USER_LED1.store(&mut user_led1, Ordering::Relaxed);

    rprintln!("done!");
}



fn start_rtos() -> ! {

    setup_tasks();
    // this should never return:
    FreeRtosUtils::start_scheduler()
    // unreachable!()

}
// fn start_rtos() -> ! {
//     rprintln!("setup rtos...");
//
//     let _rc = cmsis_rtos2::rtos_kernel_initialize();
//     let _tick_hz = cmsis_rtos2::rtos_kernel_get_tick_freq_hz();
//     let _sys_timer_hz = cmsis_rtos2::rtos_kernel_get_sys_timer_freq_hz();
//     rprintln!("tick_hz: {} sys_timer_hz: {} ", _tick_hz, _sys_timer_hz);
//
//     setup_threads();
//
//     // this should never return:
//     let rc = cmsis_rtos2::rtos_kernel_start();
//     rprintln!("kernel exit: {}", rc);
//
//     unreachable!()
// }

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- MAIN --");



    setup_peripherals();
    //this should never return:
    start_rtos()


}
