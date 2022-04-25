use crate::dynamics::joint::{JointAxesMask, JointData};
use crate::dynamics::{JointAxis, MotorModel};
use crate::math::{Point, Real};

#[cfg(feature = "dim3")]
use crate::math::UnitVector;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WheelJoint {
    data: JointData,
}

impl WheelJoint {
    #[cfg(feature = "dim3")]
    pub fn new(axis: UnitVector<Real>) -> Self {
        let mask = JointAxesMask::X
            | JointAxesMask::Y
            | JointAxesMask::Z;

        let data = JointData::default()
            .lock_axes(mask)
            .local_axis1(axis)
            .local_axis2(axis)
            .limit_axis(JointAxis::AngY, [0.0, 0.0]);
        Self { data }
    }

    pub fn data(&self) -> &JointData {
        &self.data
    }

    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Point<Real>) -> Self {
        self.data = self.data.local_anchor1(anchor1);
        self
    }

    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Point<Real>) -> Self {
        self.data = self.data.local_anchor2(anchor2);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn motor_velocity(mut self, axis: JointAxis, target_vel: Real, factor: Real) -> Self {
        self.data = self
            .data
            .motor_velocity(axis, target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    pub fn motor_position(mut self, axis: JointAxis, target_pos: Real, stiffness: Real, damping: Real) -> Self {
        self.data = self
            .data
            .motor_position(axis, target_pos, stiffness, damping);
        self
    }

    #[must_use]
    pub fn limit_axis(mut self, axis: JointAxis, limits: [Real; 2]) -> Self {
        self.data = self.data.limit_axis(axis, limits);
        self
    }

    pub fn unlimit_axis(mut self, axis: JointAxis) -> Self {
        self.data = self.data.unlimit_axis(axis);
        self
    }
}

impl Into<JointData> for WheelJoint {
    fn into(self) -> JointData {
        self.data
    }
}
