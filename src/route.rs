#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Cardinal {
    North,
    South,
    East,
    West,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Route {
    Right,
    Straight,
    Left,
}

impl Route {
    pub fn random() -> Self {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        match rng.gen_range(0..3) {
            0 => Route::Right,
            1 => Route::Straight,
            _ => Route::Left,
        }
    }
}

impl Cardinal {
    pub fn random() -> Self {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        match rng.gen_range(0..4) {
            0 => Cardinal::North,
            1 => Cardinal::South,
            2 => Cardinal::East,
            _ => Cardinal::West,
        }
    }
}
