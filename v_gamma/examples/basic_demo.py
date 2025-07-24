#!/usr/bin/env python3
"""Basic robotics demonstration example."""

import time
from robotics_suite.core.engine import SimulationEngine, SimulationConfig
from robotics_suite.scenarios.manager import ScenarioManager
from robotics_suite.scenarios.pick_place import PickPlaceConfig
from robotics_suite.utils.logger import get_logger

logger = get_logger(__name__)


def main():
    """Run a basic robotics demonstration."""
    
    logger.info("Starting basic robotics demonstration")
    
    # Create simulation configuration
    config = SimulationConfig(
        gui_enabled=True,
        real_time=False,
        debug_mode=True
    )
    
    # Initialize and run simulation
    with SimulationEngine(config) as engine:
        engine.initialize()
        
        # Create scenario manager
        manager = ScenarioManager(engine)
        
        # Create pick and place scenario
        scenario_config = PickPlaceConfig(
            name="basic_demo",
            description="Basic pick and place demonstration",
            pick_positions=[[0.5, 0.3, 0.1], [0.3, 0.5, 0.1]],
            place_positions=[[0.5, -0.3, 0.1], [0.3, -0.5, 0.1]],
            approach_height=0.2,
            grip_delay=1.0
        )
        
        scenario = manager.create_scenario("pick_place", scenario_config)
        
        # Start scenario
        manager.start_scenario("basic_demo")
        
        # Run for 30 seconds
        logger.info("Running demonstration for 30 seconds...")
        engine.run(duration=30.0)
        
        # Display final metrics
        metrics = manager.get_scenario_metrics()
        logger.info("Final metrics:")
        for key, value in metrics.items():
            if not isinstance(value, dict):
                logger.info(f"  {key}: {value}")
                
        logger.info("Demonstration complete!")


if __name__ == "__main__":
    main()