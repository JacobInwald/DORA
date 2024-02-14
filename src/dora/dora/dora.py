import interface as inter
import routing as rt
import recognition as rec

class DORA:
    
    def __init__(self) -> None:
        self.interface = inter.ROSInterface()
        # Waits for it to initilize
        while self.interface.lidar.cur_scan == None:
            self.interface.update_sensors()
            
        self.router = rt.Router(self.interface)
        self.recognition = rec.Model()
        
        self.ground_truth = rt.OccupancyMap(self.interface.gps.pos, [])
    
    def map(self) -> None:
        last_pos = self.interface.gps.pos
        next_pos = last_pos - 1
        n = 0
        max_iter = 10
        
        while not (last_pos[0] == next_pos[0] and last_pos[1] == next_pos[1]) and n < max_iter:
            
            # Update the ground truth
            cloud = rt.PointCloud(self.interface.lidar.cur_scan, self.interface.gps.pos, 1.5)
            self.ground_truth.merge(rt.OccupancyMap(last_pos, cloud))
            
            # Find next point
            last_pos = next_pos
            next_pos = self.router.nextMappingPoint(self.ground_truth)
            # Route to the next point 
            for p in self.router.route(last_pos, next_pos, self.ground_truth):
                # Move to the next point in route
                self.router.toPoint(self.interface.gps.pos, self.interface.gps.rot, p)
                # Update sensors
                for i in range(5):
                    self.interface.update_sensors()
            self.ground_truth.generate()
            self.ground_truth.show()
            n+=1
            

    def tidy(self) -> None:
        pass
    
dora = DORA()
dora.map()
dora.ground_truth.generate()
dora.ground_truth.show()
dora.tidy()