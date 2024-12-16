#![no_std] // 不使用标准库
#![cfg_attr(not(feature = "simulator"), no_main)] // 如果没有启用 "simulator" 特性，则不使用标准 main 函数

extern crate alloc; // 引入 alloc crate

slint::include_modules!(); // 包含 Slint 模块

fn create_slint_app() -> MainWindow {
    let ui = MainWindow::new().expect("Failed to load UI"); // 创建新的 UI 窗口

    let _ui_handle = ui.as_weak(); // 获取 UI 的弱引用
    ui // 返回 UI 窗口
}

#[cfg(feature = "simulator")]
fn main() -> Result<(), slint::PlatformError> {
    create_slint_app().run() // 在模拟器中运行应用程序
}

#[cfg(not(feature = "simulator"))]
#[rp_pico::entry]
fn main() -> ! {
    // 引入必要的 trait
    use cortex_m::delay::Delay;
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use embedded_hal::PwmPin;
    use fugit::RateExtU32;
    use panic_halt as _;
    use rp_pico::hal;
    use rp_pico::hal::multicore::{Multicore, Stack};
    use rp_pico::hal::pac;
    use rp_pico::hal::prelude::*;

    // -------- 设置分配器 --------
    const HEAP_SIZE: usize = 200 * 1024; // 定义堆大小
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE]; // 定义堆数组
    #[global_allocator]
    static ALLOCATOR: embedded_alloc::Heap = embedded_alloc::Heap::empty(); // 定义全局分配器
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) } // 初始化分配器

    // -------- 设置外设 --------
    let mut pac = pac::Peripherals::take().unwrap(); // 获取外设
    let core = pac::CorePeripherals::take().unwrap(); // 获取核心外设
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG); // 初始化看门狗

    static mut CORE1_STACK: Stack<4096> = Stack::new(); // 定义第二个核心的栈

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap(); // 初始化时钟和 PLL

    let sys_freq = clocks.system_clock.freq().to_Hz(); // 获取系统时钟频率

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().raw()); // 初始化延迟
    let mut sio = hal::sio::Sio::new(pac.SIO); // 初始化 SIO

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo); // 初始化多核
    let cores = mc.cores(); // 获取核心
    let core1 = &mut cores[1]; // 获取第二个核心

    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS); // 初始化引脚

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS); // 初始化定时器

    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut beep_pin = pins.vbus_detect.into_push_pull_output();
    let led_pin = pins.led.into_mode::<hal::gpio::FunctionPwm>();
    let mut pwm = pwm_slices.pwm4;

    pwm.set_ph_correct();
    pwm.enable();

    pwm.channel_b.output_to(led_pin);
    pwm.channel_b.set_duty(0); // 初始设置为 0% 亮度

    let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>(); // 设置 SPI 时钟引脚
    let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>(); // 设置 SPI MOSI 引脚
    let _spi_miso = pins.gpio16.into_mode::<hal::gpio::FunctionSpi>(); // 设置 SPI MISO 引脚

    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI0); // 初始化 SPI
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        62_500_000.Hz(),
        &embedded_hal::spi::MODE_3,
    ); // 配置 SPI
    let spi = shared_bus::BusManagerSimple::new(spi); // 创建 SPI 总线管理器

    let bl = pins.b_power_save.into_push_pull_output(); // 设置背光引脚
    let rst = pins.gpio2.into_push_pull_output(); // 设置复位引脚
    let dc = pins.gpio6.into_push_pull_output(); // 设置数据/命令引脚
    let cs = pins.gpio17.into_push_pull_output(); // 设置片选引脚
    let di = display_interface_spi::SPIInterface::new(spi.acquire_spi(), dc, cs); // 创建 SPI 接口
    let mut display = st7789::ST7789::new(di, Some(rst), Some(bl), 240, 240); // 初始化显示屏

    // -------- 设置按钮 --------
    let button_dn = pins.gpio20.into_pull_up_input();
    let button_up = pins.gpio7.into_pull_up_input();
    let button_lt = pins.gpio21.into_pull_up_input();
    let button_rt = pins.gpio22.into_pull_up_input();
    let button_a = pins.gpio11.into_pull_up_input();
    let button_b = pins.gpio10.into_pull_up_input();
    let button_x = pins.gpio3.into_pull_up_input();
    let button_y = pins.gpio28.into_pull_up_input();

    display.init(&mut delay).unwrap(); // 初始化显示屏
    display.set_orientation(st7789::Orientation::Portrait).unwrap(); // 设置显示屏方向

    // -------- 设置 Slint 后端 --------
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default()); // 创建软件渲染窗口
    window.set_size(slint::PhysicalSize::new(240, 240)); // 设置窗口大小
    let platform = MyPlatform { window: window.clone(), timer };
    slint::platform::set_platform(alloc::boxed::Box::new(platform)).unwrap();

    struct MyPlatform {
        window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow>, // 窗口
        timer: hal::Timer,                                                                // 定时器
    }

    impl slint::platform::Platform for MyPlatform {
        fn create_window_adapter(
            &self,
        ) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError>
        {
            Ok(self.window.clone()) // 创建窗口适配器
        }
        fn duration_since_start(&self) -> core::time::Duration {
            core::time::Duration::from_micros(self.timer.get_counter().ticks()) // 获取启动以来的持续时间
        }
    }

    // -------- 配置 UI --------
    // （需要在调用 slint::platform::set_platform 之后完成）
    let ui = create_slint_app(); // 创建 Slint 应用程序

    // -------- 事件循环 --------
    let mut line = [slint::platform::software_renderer::Rgb565Pixel(0); 320]; // 初始化行缓冲区

    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        let core = unsafe { pac::CorePeripherals::steal() };
        // Set up the delay for the second core.
        let mut delay = Delay::new(core.SYST, sys_freq);
        loop {
            delay.delay_ms(1000);
            beep_pin.set_high().unwrap();
            delay.delay_ms(1000);
            beep_pin.set_low().unwrap();
        }
    }); // 启动第二个核心

    loop {
        slint::platform::update_timers_and_animations(); // 更新计时器和动画
        window.draw_if_needed(|renderer| {
            use embedded_graphics_core::prelude::*;
            struct DisplayWrapper<'a, T>(
                &'a mut T,
                &'a mut [slint::platform::software_renderer::Rgb565Pixel],
            );
            impl<T: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>
                slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, T>
            {
                type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
                fn process_line(
                    &mut self,
                    line: usize,
                    range: core::ops::Range<usize>,
                    render_fn: impl FnOnce(&mut [Self::TargetPixel]),
                ) {
                    let rect = embedded_graphics_core::primitives::Rectangle::new(
                        Point::new(range.start as _, line as _),
                        Size::new(range.len() as _, 1),
                    ); // 创建矩形
                    render_fn(&mut self.1[range.clone()]); // 渲染行
                    self.0
                        .fill_contiguous(
                            &rect,
                            self.1[range.clone()].iter().map(|p| {
                                embedded_graphics_core::pixelcolor::raw::RawU16::new(p.0).into()
                            }),
                        )
                        .map_err(drop)
                        .unwrap(); // 填充矩形
                }
            }
            renderer.render_by_line(DisplayWrapper(&mut display, &mut line)); // 渲染行
        });

        if button_y.is_low().unwrap() {
            delay.delay_ms(50);
            if button_y.is_low().unwrap() {
                ui.set_night_mode(ui.get_night_mode() ^ true);
            }
        }

        //active page is from 0 to 2
        if button_dn.is_low().unwrap() {
            delay.delay_ms(50);
            if button_dn.is_low().unwrap() {
                ui.set_active_page((ui.get_active_page() + 1) % 3);
            }
        }

        if button_up.is_low().unwrap() {
            delay.delay_ms(50);
            if button_up.is_low().unwrap() {
                ui.set_active_page((ui.get_active_page() + 2) % 3);
            }
        }

        let selected: [bool; 2] = [ui.get_beep_slected(), ui.get_led_slected()];
        //when slected[0] is true,press button_rt or button_lt to make selected[0] false and selected[1] true
        //when slected[1] is true,press button_lt or button_lf to make selected[1] false and selected[0] true

        if selected[0] {
            if button_rt.is_low().unwrap() {
                delay.delay_ms(50);
                if button_rt.is_low().unwrap() {
                    ui.set_beep_slected(false);
                    ui.set_led_slected(true);
                }
            }
            if button_lt.is_low().unwrap() {
                delay.delay_ms(50);
                if button_lt.is_low().unwrap() {
                    ui.set_beep_slected(false);
                    ui.set_led_slected(true);
                }
            }
            if button_a.is_low().unwrap() {
                delay.delay_ms(50);
                if button_a.is_low().unwrap() {
                    // 增加音量
                    ui.set_beep_volume((ui.get_beep_volume() + 1.0) % 101.0);
                }
            }
            if button_b.is_low().unwrap() {
                delay.delay_ms(50);
                if button_b.is_low().unwrap() {
                    // 降低音量
                    ui.set_beep_volume((ui.get_beep_volume() + 100.0) % 101.0);
                }
            }
        }

        if selected[1] {
            if button_rt.is_low().unwrap() {
                delay.delay_ms(50);
                if button_rt.is_low().unwrap() {
                    ui.set_led_slected(false);
                    ui.set_beep_slected(true);
                }
            }
            if button_lt.is_low().unwrap() {
                delay.delay_ms(50);
                if button_lt.is_low().unwrap() {
                    ui.set_led_slected(false);
                    ui.set_beep_slected(true);
                }
            }
            if button_a.is_low().unwrap() {
                delay.delay_ms(50);
                if button_a.is_low().unwrap() {
                    // 增加亮度
                    let current_duty = pwm.channel_b.get_duty();
                    let max_duty = pwm.channel_b.get_max_duty();
                    if current_duty + (max_duty / 10) <= max_duty {
                        pwm.channel_b.set_duty(current_duty + (max_duty / 10));
                        ui.set_led_brightness(
                            (current_duty + (max_duty / 10)) as f32 / max_duty as f32,
                        );
                    } else {
                        pwm.channel_b.set_duty(max_duty);
                        ui.set_led_brightness(1.0);
                    }
                }
            }
            if button_b.is_low().unwrap() {
                delay.delay_ms(50);
                if button_b.is_low().unwrap() {
                    // 降低亮度
                    let current_duty = pwm.channel_b.get_duty();
                    let max_duty = pwm.channel_b.get_max_duty();
                    if current_duty >= (max_duty / 10) {
                        pwm.channel_b.set_duty(current_duty - (max_duty / 10));
                        ui.set_led_brightness(
                            (current_duty - (max_duty / 10)) as f32 / max_duty as f32,
                        );
                    } else {
                        pwm.channel_b.set_duty(0);
                        ui.set_led_brightness(0.0);
                    }
                }
            }
        }

        if button_x.is_low().unwrap() {
            delay.delay_ms(50);
            if button_x.is_low().unwrap() {
                ui.set_orbiter_value((ui.get_orbiter_value() + 1) % 3);
            }
        }

        if window.has_active_animations() {
            continue; // 如果有活动动画，则继续循环
        }

        // TODO: 我们可以通过进入睡眠状态来节省电池
        // 或直到下一个触摸中断，以先到者为准
        // cortex_m::asm::wfe();
    }
}
