use crate::vehicle::Vehicle;

#[derive(Debug, Clone, Default)]
pub struct Statistics {
    pub total_vehicles: usize,
    pub max_velocity: f64,
    pub min_velocity: f64,
    pub max_time: f64,
    pub min_time: f64,
    pub close_calls: u32,
}

impl Statistics {
    pub fn compute(vehicles: &[Vehicle], close_calls: u32) -> Self {
        let max_velocity = vehicles
            .iter()
            .map(|v| v.max_velocity)
            .fold(0.0_f64, f64::max);

        let min_velocity = vehicles
            .iter()
            .map(|v| v.min_velocity)
            .filter(|v| *v < f64::MAX)
            .fold(f64::INFINITY, f64::min);

        let min_velocity = if min_velocity == f64::INFINITY { 0.0 } else { min_velocity };

        let max_time = vehicles
            .iter()
            .map(|v| v.time_in_intersection)
            .fold(0.0_f64, f64::max);

        let min_time = if vehicles.is_empty() {
            0.0
        } else {
            vehicles
                .iter()
                .map(|v| v.time_in_intersection)
                .fold(f64::INFINITY, f64::min)
        };

        Self {
            total_vehicles: vehicles.len(),
            max_velocity,
            min_velocity,
            max_time,
            min_time,
            close_calls,
        }
    }
}
