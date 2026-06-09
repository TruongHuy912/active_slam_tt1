from typing import Dict


def slam_profile_summary(profile_name: str) -> Dict[str, float]:
    if profile_name == "simulation_fast_map":
        return {
            "map_update_interval": 1.0,
            "minimum_time_interval": 0.2,
            "minimum_travel_distance": 0.2,
            "minimum_travel_heading": 0.2,
        }
    return {
        "map_update_interval": 2.0,
        "minimum_time_interval": 0.5,
        "minimum_travel_distance": 0.5,
        "minimum_travel_heading": 0.5,
    }
