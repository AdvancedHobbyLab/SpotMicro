from ServoController import ServoController
import Locomotion
import numpy as np
import asyncio

controller = ServoController()

ports = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
offsets = np.loadtxt("config.csv", delimiter=",", dtype=int)
#offsets = np.zeros(12)

servos = controller.load_servos(ports, offsets)
locomotion = Locomotion.Locomotion(servos)

# Configure leg servos to use a wider range
for i in range(len(servos)):
    servos[i].set_min_angle(22.5)
    servos[i].set_min_pulse(.5 / ((1/50.0)/4096 * 1e3))
    servos[i].set_max_angle(157.5)
    servos[i].set_max_pulse(2.5 / ((1/50.0)/4096 * 1e3))

async def TestRoutine():
    await asyncio.sleep(1)
    
    # Start Trotting
    locomotion.toggle_standing()
    await asyncio.sleep(1)
    
    # Walk Forward
    locomotion.set_forward_factor(1.0)
    await asyncio.sleep(2)
    
    # Trot
    locomotion.set_forward_factor(0.0)
    await asyncio.sleep(1)
    
    # Walk Backwards
    locomotion.set_forward_factor(-1.0)
    await asyncio.sleep(2)

    # Trot
    locomotion.set_forward_factor(0.0)
    await asyncio.sleep(1)

    locomotion.toggle_standing()
    locomotion.Shutdown()

loop = asyncio.get_event_loop()

async def main():
    locomotion_task = asyncio.create_task(locomotion.Run())
    test_task = asyncio.create_task(TestRoutine())
    
    await locomotion_task
    await test_task

asyncio.run(main())