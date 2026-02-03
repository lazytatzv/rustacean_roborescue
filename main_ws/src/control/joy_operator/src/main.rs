use rclrs::*;
use sensor_msgs;
use anyhow::Result;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("joy_operator")?;

    let subscription = node.create_subscription(
        "/joy",
        |msg: sensor_msgs::msg::Joy| {
            println!("Pressed: {}", msg.buttons[0]);
        }
    )?;

    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}
