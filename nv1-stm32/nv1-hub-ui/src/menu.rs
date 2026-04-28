use alloc::rc::Rc;
use alloc::{boxed::Box, vec::Vec};
use core::cell::RefCell;

use embedded_graphics::prelude::Primitive;
use embedded_graphics::primitives::{Circle, Line, Triangle};
use embedded_graphics::Drawable;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::{DrawTarget, Point, Size},
    primitives::{PrimitiveStyle, Rectangle},
};

use crate::elements::{Element, ElementInfo};

pub trait Menu<T>
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T) -> Result<(), T::Error>;
    fn event(&mut self, event: &crate::Event);
}

pub struct ListMenuOption {
    pub position: Point,
    pub size: Size,
    pub vertical_num: usize,
    pub element_margin: usize,
    pub cursor_line_len: i32,
}

pub struct ListMenu<T> {
    option: ListMenuOption,
    pub elements: Vec<Box<dyn Element<T>>>,
    selected_element: usize,
    scroll: usize,
    entering_cursor: bool,
}

impl<T> ListMenu<T>
where
    T: DrawTarget<Color = BinaryColor>,
{
    pub fn new(elements: Vec<Box<dyn Element<T>>>, option: ListMenuOption) -> Self {
        ListMenu {
            option,
            elements,
            selected_element: 0,
            scroll: 0,
            entering_cursor: false,
        }
    }

    fn draw_cursor(&self, display: &mut T) -> Result<(), T::Error> {
        let mut position = self.calculate_element_position(self.selected_element);
        position.x -= self.option.element_margin as i32;
        position.y -= self.option.element_margin as i32;

        let mut size = self.calculate_element_size();
        size.width += self.option.element_margin as u32 * 2;
        size.height += self.option.element_margin as u32 * 2;

        let style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

        if self.entering_cursor {
            Rectangle::new(position, size)
                .into_styled(style.clone())
                .draw(display)?;
            return Ok(());
        }

        let position_left_top = position;
        let position_left_top1 = Point::new(
            position_left_top.x + self.option.cursor_line_len,
            position_left_top.y,
        );
        let position_left_top2 = Point::new(
            position_left_top.x,
            position_left_top.y + self.option.cursor_line_len,
        );
        Line::new(position_left_top, position_left_top1)
            .into_styled(style.clone())
            .draw(display)?;
        Line::new(position_left_top, position_left_top2)
            .into_styled(style.clone())
            .draw(display)?;

        let position_right_top = Point::new(position.x + size.width as i32 - 1, position.y);
        let position_right_top1 = Point::new(
            position_right_top.x - self.option.cursor_line_len,
            position_right_top.y,
        );
        let position_right_top2 = Point::new(
            position_right_top.x,
            position_right_top.y + self.option.cursor_line_len,
        );
        Line::new(position_right_top, position_right_top1)
            .into_styled(style.clone())
            .draw(display)?;
        Line::new(position_right_top, position_right_top2)
            .into_styled(style.clone())
            .draw(display)?;

        let position_left_bottom = Point::new(position.x, position.y + size.height as i32 - 1);
        let position_left_bottom1 = Point::new(
            position_left_bottom.x + self.option.cursor_line_len,
            position_left_bottom.y,
        );
        let position_left_bottom2 = Point::new(
            position_left_bottom.x,
            position_left_bottom.y - self.option.cursor_line_len,
        );
        Line::new(position_left_bottom, position_left_bottom1)
            .into_styled(style.clone())
            .draw(display)?;
        Line::new(position_left_bottom, position_left_bottom2)
            .into_styled(style.clone())
            .draw(display)?;

        let position_right_bottom = Point::new(
            position.x + size.width as i32 - 1,
            position.y + size.height as i32 - 1,
        );
        let position_right_bottom1 = Point::new(
            position_right_bottom.x - self.option.cursor_line_len,
            position_right_bottom.y,
        );
        let position_right_bottom2 = Point::new(
            position_right_bottom.x,
            position_right_bottom.y - self.option.cursor_line_len,
        );
        Line::new(position_right_bottom, position_right_bottom1)
            .into_styled(style.clone())
            .draw(display)?;
        Line::new(position_right_bottom, position_right_bottom2)
            .into_styled(style.clone())
            .draw(display)?;

        Ok(())
    }

    fn calculate_element_position(&self, index: usize) -> Point {
        let height = self.option.size.height / self.option.vertical_num as u32;
        let margin = self.option.element_margin as i32;

        Point::new(
            self.option.position.x + margin,
            self.option.position.y + margin + height as i32 * (index as i32 - self.scroll as i32),
        )
    }

    fn calculate_element_size(&self) -> Size {
        Size::new(
            self.option.size.width - self.option.element_margin as u32 * 2,
            self.option.size.height / self.option.vertical_num as u32
                - self.option.element_margin as u32 * 2,
        )
    }
}

impl<T> Menu<T> for ListMenu<T>
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T) -> Result<(), T::Error> {
        for (i, element) in self.elements.iter().enumerate() {
            let position = self.calculate_element_position(i);
            let size = self.calculate_element_size();

            let info = ElementInfo {
                selected: i == self.selected_element,
                position,
                size,
            };
            element.draw(display, info)?;
        }

        self.draw_cursor(display)?;
        Ok(())
    }

    fn event(&mut self, event: &crate::Event) {
        let positions: Vec<_> = (0..self.elements.len())
            .map(|i| self.calculate_element_position(i))
            .collect();

        let size = self.calculate_element_size();

        let mut entering = false;

        for (i, element) in self.elements.iter_mut().enumerate() {
            let position = positions[i];
            let info = ElementInfo {
                selected: i == self.selected_element,
                position,
                size,
            };

            if i == self.selected_element {
                entering = element.event(event, info);
            }
        }

        self.entering_cursor = entering;

        if entering {
            return;
        } else {
            match event {
                crate::Event::KeyDown(crate::EventKey::Up) => {
                    if self.selected_element > 0 {
                        self.selected_element -= 1;
                    }
                    if self.selected_element < self.scroll {
                        self.scroll -= 1;
                    }
                }
                crate::Event::KeyDown(crate::EventKey::Down) => {
                    if self.selected_element < self.elements.len() - 1 {
                        self.selected_element += 1;
                    }
                    if self.selected_element >= self.option.vertical_num + self.scroll {
                        self.scroll += 1;
                    }
                }
                crate::Event::KeyDown(crate::EventKey::Enter) => {}
                _ => {}
            }
        }
    }
}

pub struct RobotStatusMenuOption {
    pub position: Point,
    pub size: Size,
    /// Wheel speed magnitude (in the same units as the source) at which the
    /// rotation arrow reaches its maximum length. Speeds beyond this clamp.
    pub max_wheel_speed: f32,
}

/// Per-wheel screen geometry. `offset` is the unit vector from the chassis
/// centre to the wheel position (in screen coordinates: +x right, +y down).
/// `tangent` is the unit vector along the wheel's rolling axis for a
/// positive motor speed; the arrow is drawn `sgn(speed) * tangent` from the
/// wheel centre.
struct WheelGeometry {
    offset: (f32, f32),
    tangent: (f32, f32),
}

const SQRT_2_2: f32 = 0.707_106_77;

/// Wheels indexed as motor1..motor4. Chassis angles 45°, 315°, 225°, 135°
/// (matching `WHEEL{1..4}_ANGLE` in `nv1-hub-core::constants`). Body +y
/// (forward) maps to screen -y (up), body +x (right) maps to screen +x.
/// Tangent = positive rolling direction in body frame, projected to screen.
const WHEEL_GEOMETRY: [WheelGeometry; 4] = [
    // Motor 1 — chassis 45° (front-right) → top-right
    WheelGeometry {
        offset: (SQRT_2_2, -SQRT_2_2),
        tangent: (-SQRT_2_2, -SQRT_2_2),
    },
    // Motor 2 — chassis 315° (back-right) → bottom-right
    WheelGeometry {
        offset: (SQRT_2_2, SQRT_2_2),
        tangent: (SQRT_2_2, -SQRT_2_2),
    },
    // Motor 3 — chassis 225° (back-left) → bottom-left
    WheelGeometry {
        offset: (-SQRT_2_2, SQRT_2_2),
        tangent: (SQRT_2_2, SQRT_2_2),
    },
    // Motor 4 — chassis 135° (front-left) → top-left
    WheelGeometry {
        offset: (-SQRT_2_2, -SQRT_2_2),
        tangent: (-SQRT_2_2, SQRT_2_2),
    },
];

/// Layout of the status menu, all radii measured from the chassis centre and
/// scaled so the IR triangle just fits inside `size`. Numerator/denominator
/// pairs keep the computation in i32.
struct StatusLayout {
    centre: Point,
    chassis_radius: i32,
    wheel_half_len: i32,
    wheel_thickness: u32,
    arrow_base_radius: i32,
    arrow_max_len: i32,
    arrow_head_len: i32,
    ball_orbit_radius: i32,
    ball_triangle_height: i32,
    ball_triangle_half_base: i32,
}

impl StatusLayout {
    fn from_option(opt: &RobotStatusMenuOption) -> Self {
        let max_size = opt.size.width.min(opt.size.height) as i32;
        // Box centre — Circle::new(top_left, diameter) places its centre at
        // top_left + (diameter/2, diameter/2).
        let centre = Point::new(opt.position.x + max_size / 2, opt.position.y + max_size / 2);

        // Chassis circle takes ~9/16 of the available diameter so wheels,
        // rotation arrows, and the orbiting ball triangle each get their
        // own concentric ring outside it.
        let chassis_radius = (max_size * 9) / 32;
        let arrow_base_radius = chassis_radius + 5;
        let arrow_max_len = (max_size / 10).max(4);
        let ball_orbit_radius = max_size / 2 - 6;

        Self {
            centre,
            chassis_radius,
            wheel_half_len: 6,
            wheel_thickness: 3,
            arrow_base_radius,
            arrow_max_len,
            arrow_head_len: 3,
            ball_orbit_radius,
            ball_triangle_height: 4,
            ball_triangle_half_base: 3,
        }
    }
}

pub struct RobotStatusMenu {
    option: RobotStatusMenuOption,
    wheel_speeds: Rc<RefCell<[f32; 4]>>,
    ball_angle: Rc<RefCell<f32>>,
}

impl RobotStatusMenu {
    pub fn new(
        option: RobotStatusMenuOption,
        wheel_speeds: Rc<RefCell<[f32; 4]>>,
        ball_angle: Rc<RefCell<f32>>,
    ) -> Self {
        RobotStatusMenu {
            option,
            wheel_speeds,
            ball_angle,
        }
    }
}

fn scale_offset(unit: (f32, f32), len: i32) -> (i32, i32) {
    ((unit.0 * len as f32) as i32, (unit.1 * len as f32) as i32)
}

fn draw_wheel_arrow<T>(
    display: &mut T,
    layout: &StatusLayout,
    radial: (f32, f32),
    tangent: (f32, f32),
    speed: f32,
    max_speed: f32,
    style: PrimitiveStyle<BinaryColor>,
) -> Result<(), T::Error>
where
    T: DrawTarget<Color = BinaryColor>,
{
    if !speed.is_finite() || max_speed <= 0.0 {
        return Ok(());
    }

    let normalised = (speed / max_speed).clamp(-1.0, 1.0);
    let length = (normalised.abs() * layout.arrow_max_len as f32) as i32;
    if length < 2 {
        return Ok(());
    }

    let dir = if normalised >= 0.0 {
        tangent
    } else {
        (-tangent.0, -tangent.1)
    };

    // Arrow shaft sits on its own concentric ring outside the wheel marker.
    let (rdx, rdy) = scale_offset(radial, layout.arrow_base_radius);
    let base = Point::new(layout.centre.x + rdx, layout.centre.y + rdy);

    let (dx, dy) = scale_offset(dir, length);
    let tip = Point::new(base.x + dx, base.y + dy);
    Line::new(base, tip).into_styled(style).draw(display)?;

    // Two-line chevron at the tip, rotated ±135° from the arrow direction.
    let (head1_x, head1_y) = (
        -SQRT_2_2 * dir.0 - SQRT_2_2 * dir.1,
        SQRT_2_2 * dir.0 - SQRT_2_2 * dir.1,
    );
    let (head2_x, head2_y) = (
        -SQRT_2_2 * dir.0 + SQRT_2_2 * dir.1,
        -SQRT_2_2 * dir.0 - SQRT_2_2 * dir.1,
    );
    let (h1dx, h1dy) = scale_offset((head1_x, head1_y), layout.arrow_head_len);
    let (h2dx, h2dy) = scale_offset((head2_x, head2_y), layout.arrow_head_len);
    Line::new(tip, Point::new(tip.x + h1dx, tip.y + h1dy))
        .into_styled(style)
        .draw(display)?;
    Line::new(tip, Point::new(tip.x + h2dx, tip.y + h2dy))
        .into_styled(style)
        .draw(display)?;

    Ok(())
}

/// Draw the IR ball direction as a filled triangle on the outer orbit ring,
/// pointing radially outward at `body_angle` (radians, body frame: +x right,
/// +y forward).
fn draw_ball_triangle<T>(
    display: &mut T,
    layout: &StatusLayout,
    body_angle: f32,
    style: PrimitiveStyle<BinaryColor>,
) -> Result<(), T::Error>
where
    T: DrawTarget<Color = BinaryColor>,
{
    if !body_angle.is_finite() {
        return Ok(());
    }

    // Body → screen: +y body (forward) maps to -y screen (up).
    let cos = libm::cosf(body_angle);
    let sin = libm::sinf(body_angle);
    let radial = (cos, -sin);
    let tangent = (-radial.1, radial.0);

    let (rx, ry) = scale_offset(radial, layout.ball_orbit_radius);
    let base_centre = Point::new(layout.centre.x + rx, layout.centre.y + ry);

    let (tx, ty) = scale_offset(
        radial,
        layout.ball_orbit_radius + layout.ball_triangle_height,
    );
    let tip = Point::new(layout.centre.x + tx, layout.centre.y + ty);

    let (px, py) = scale_offset(tangent, layout.ball_triangle_half_base);
    let base_l = Point::new(base_centre.x - px, base_centre.y - py);
    let base_r = Point::new(base_centre.x + px, base_centre.y + py);

    Triangle::new(tip, base_l, base_r)
        .into_styled(style)
        .draw(display)?;

    Ok(())
}

impl<T> Menu<T> for RobotStatusMenu
where
    T: DrawTarget<Color = BinaryColor>,
{
    fn draw(&self, display: &mut T) -> Result<(), <T as DrawTarget>::Error> {
        let layout = StatusLayout::from_option(&self.option);

        let chassis_diameter = (layout.chassis_radius as u32) * 2;
        let chassis_top_left = Point::new(
            layout.centre.x - layout.chassis_radius,
            layout.centre.y - layout.chassis_radius,
        );
        Circle::new(chassis_top_left, chassis_diameter)
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
            .draw(display)?;

        let wheel_style = PrimitiveStyle::with_stroke(BinaryColor::On, layout.wheel_thickness);
        let arrow_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
        let fill_style = PrimitiveStyle::with_fill(BinaryColor::On);

        let speeds = *self.wheel_speeds.borrow();

        for (geom, &speed) in WHEEL_GEOMETRY.iter().zip(speeds.iter()) {
            // Wheels sit on the chassis rim, tangent line centred at radius
            // `chassis_radius`.
            let wheel_centre = Point::new(
                layout.centre.x + (geom.offset.0 * layout.chassis_radius as f32) as i32,
                layout.centre.y + (geom.offset.1 * layout.chassis_radius as f32) as i32,
            );

            let (tdx, tdy) = scale_offset(geom.tangent, layout.wheel_half_len);
            let line_a = Point::new(wheel_centre.x - tdx, wheel_centre.y - tdy);
            let line_b = Point::new(wheel_centre.x + tdx, wheel_centre.y + tdy);
            Line::new(line_a, line_b)
                .into_styled(wheel_style)
                .draw(display)?;

            draw_wheel_arrow(
                display,
                &layout,
                geom.offset,
                geom.tangent,
                speed,
                self.option.max_wheel_speed,
                arrow_style,
            )?;
        }

        draw_ball_triangle(display, &layout, *self.ball_angle.borrow(), fill_style)?;

        Ok(())
    }

    fn event(&mut self, _event: &crate::Event) {}
}
