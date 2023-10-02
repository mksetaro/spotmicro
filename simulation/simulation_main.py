import time
import simulation
import os


def main():
    try:
        sim_parameters = simulation.SimulationParameters()
        sim_client = simulation.SimulationClient(sim_parameters)
        sim_client.start()
        while True:
            sim_client.step()
    except KeyboardInterrupt:
        pass
    finally:
        sim_client.shutdown()


if __name__ == "__main__":
    main()
