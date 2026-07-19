use crate::messages::Input;
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub(crate) struct MessageRecorder<T> {
    pub input_msg: Input<T>,
    pub samples: Vec<T>,
}

impl<T> Default for MessageRecorder<T> {
    fn default() -> Self {
        Self {
            input_msg: Input::default(),
            samples: Vec::new(),
        }
    }
}

impl<T> Module for MessageRecorder<T>
where
    T: Clone + Default + Send + Sync,
{
    fn init(&mut self) {
        self.samples.clear();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.samples.push(self.input_msg.read());
    }
}
