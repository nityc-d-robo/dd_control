use std::f64::consts::PI;

use motor_lib::{init_usb_handle, md};
use safe_drive::{
    context::Context, error::DynError, logger::Logger, msg::common_interfaces::geometry_msgs, pr_info,
};

struct PIDController {
    sv: f64, // 目標値
    kp: f64, // 比例項のゲイン
}
impl PIDController {
    fn new() -> PIDController {
        return PIDController { sv: 0.0, kp: 0.0 };
    }
    /// 目標値を設定する
    fn set_reference(&mut self, val: f64) {
        self.sv = val;
        return;
    }

    /// 比例ゲインを設定する
    fn set_proportional_gain(&mut self, val: f64) {
        self.kp = val;
        return;
    }

    /// 次の入力値を求める
    fn calc(&mut self, pv: f64) -> f64 {
        let term_p = self.kp * (self.sv - pv);
        let mv = term_p;
        return mv;
    }
}

fn main() -> Result<(), DynError>{
    const WHEEL_DIR: f64 = 0.06;

    let ctx = Context::new()?;
    let node = ctx.create_node("dd_control", None, Default::default())?;
    let subscriber = node.create_subscriber::<geometry_msgs::msg::Twist>("cmd_vel", None)?;

    let handle = init_usb_handle(0x483, 0x5740, 0);

    let mut pid_left = PIDController::new();
    let mut pid_right = PIDController::new();
    
    pid_left.set_proportional_gain(16.0); // 制御器の比例ゲインを設定する
    pid_right.set_proportional_gain(16.0); // 制御器の比例ゲインを設定する

    let mut selelctor = ctx.create_selector()?;

    selelctor.add_subscriber(
        subscriber, 
        Box::new(move |msg| {
            let logger = Logger::new("dd_control");

            let left_motor_rpm = md::receive_status(&handle, 0x200, 1).speed / 19;
            let right_motor_rpm = md::receive_status(&handle, 0x200, 2).speed / 19;

            let y = msg.linear.y;
            let r = msg.angular.z;

            let left_target_speed = y + (if y >= 0.0 {1.0} else {-1.0}) * r;
            let right_target_speed = y - (if y >= 0.0 {1.0} else {-1.0}) * r;

            let left_target_rpm = (left_target_speed / (2.0 * PI * WHEEL_DIR)) * 60.0;
            let right_target_rpm = (right_target_speed / (2.0 * PI * WHEEL_DIR)) * 60.0;

            pid_left.set_reference(left_target_rpm); // 制御器の目標値を設定する
            pid_right.set_reference(right_target_rpm); // 制御器の目標値を設定する
            
            let left_next = pid_left.calc(left_motor_rpm as f64);
            let right_next = pid_right.calc(right_motor_rpm as f64);

            pr_info!(logger, "左 目標の回転数: {}, 現在の回転数: {}, 入力する電流値: {}", left_target_rpm, left_motor_rpm, left_next);
            pr_info!(logger, "右 目標の回転数: {}, 現在の回転数: {}, 入力する電流値: {}", right_target_rpm, right_motor_rpm, right_next);

            md::send_current(&handle, 0x200, 1, left_next as i16);
            md::send_current(&handle, 0x200, 2, right_next as i16);
        }),
    );

    loop {
        selelctor.wait()?;
    }
}
