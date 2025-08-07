use nv1_msg::hub::HSV;

pub const LOOP_US: u64 = 1000;
pub const LINE_OVER_CENTER_THRESHOLD: f32 = 140_f32.to_radians();
pub const HEAP_SIZE: usize = 2048;

pub const WHEEL_R: f32 = 25.0 / 1000.0;
pub const THREAD: f32 = 108.0 / 1000.0;

pub const LED_COUNT: usize = 32;

pub const DEFAULT_LINE_THRESHOLD: f32 = 0.12;
pub const DEFAULT_HAVE_BALL_THRESHOLD: u16 = 800;
pub const DEFAULT_ROBOT_SPEED_MULTIPLIER: f32 = 1.5;

pub const DEFAULT_OPENCV_GOAL_BLUE: HSV = HSV {
    h_min: 0,
    h_max: 110,
    s_min: 100,
    s_max: 255,
    v_min: 0,
    v_max: 255,
};

pub const DEFAULT_OPENCV_GOAL_YELLOW: HSV = HSV {
    h_min: 50,
    h_max: 120,
    s_min: 100,
    s_max: 255,
    v_min: 100,
    v_max: 255,
};

pub const SPREAD_PATTERN: [usize; 32] = [
    0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 5, 5, 5, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0,
    0, 0,
];

pub const FLASH_SETTINGS_ADDRESS: u32 = 0x6_0000;
pub const FLASH_SETTINGS_SIZE: u32 = 128 * 1024;
pub const SETTINGS_BUFFER_SIZE: usize = 128;

pub const UART_JETSON_BAUDRATE: u32 = 2_000_000;
pub const UART_MD_BAUDRATE: u32 = 2_000_000;

pub const ADC_RESOLUTION: f32 = 4096.0;
pub const LINE_SENSORS_COUNT: usize = 32;
pub const IR_SENSORS_COUNT: usize = 16;

pub const LINE_SPEED_MULTIPLIER: f32 = 1.5;
pub const ROTATION_PID_P: f32 = 7.0;
pub const ROTATION_PID_LIMIT: f32 = 100.0;