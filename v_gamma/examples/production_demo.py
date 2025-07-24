#!/usr/bin/env python3
"""Production line demonstration example."""

import time
from robotics_suite.core.engine import SimulationEngine, SimulationConfig
from robotics_suite.scenarios.manager import ScenarioManager
from robotics_suite.scenarios.production_line import ProductionLineConfig
from robotics_suite.utils.logger import get_logger

logger = get_logger(__name__)


def main():
    """Run a production line demonstration."""
    
    logger.info("Starting production line demonstration")
    
    # Create simulation configuration
    config = SimulationConfig(
        gui_enabled=True,
        real_time=False,
        debug_mode=False  # Less verbose for production demo
    )
    
    # Initialize and run simulation
    with SimulationEngine(config) as engine:
        engine.initialize()
        
        # Create scenario manager
        manager = ScenarioManager(engine)
        
        # Create production line scenario
        scenario_config = ProductionLineConfig(
            name="production_demo",
            description="Advanced production line demonstration",
            station_count=4,
            products_per_cycle=8,
            conveyor_speed=0.15,
            cycle_time=45.0,
            quality_check_enabled=True
        )
        
        scenario = manager.create_scenario("production_line", scenario_config)
        
        # Start scenario
        manager.start_scenario("production_demo")
        
        # Run for specified cycle time
        logger.info(f"Running production line for {scenario_config.cycle_time} seconds...")
        engine.run(duration=scenario_config.cycle_time)
        
        # Display comprehensive metrics
        metrics = manager.get_scenario_metrics()
        
        logger.info("=== Production Line Results ===")
        logger.info(f"Products Processed: {metrics.get('products_processed', 0)}")
        logger.info(f"Quality Passed: {metrics.get('quality_passed', 0)}")
        logger.info(f"Efficiency: {metrics.get('efficiency', 0):.1f}%")
        logger.info(f"Quality Rate: {metrics.get('quality_rate', 0):.1f}%")
        logger.info(f"Throughput: {metrics.get('throughput', 0):.2f} products/min")
        logger.info(f"Total Time: {metrics.get('total_time', 0):.1f} seconds")
        
        # Station status
        station_states = metrics.get('station_states', {})
        if station_states:
            logger.info("Station States:")
            for station, state in station_states.items():
                logger.info(f"  {station}: {state}")
                
        logger.info("Production demonstration complete!")


if __name__ == "__main__":
    main()