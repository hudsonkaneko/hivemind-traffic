"""SUMO pose authority for a straight-road sensor-control experiment."""
import uuid
import traci


class LiveTraffic:
    def __init__(self,binary,config,*,mode,speed,gap,seed):
        self.connection=None
        self.mode=mode
        try:
            label='lidar-'+uuid.uuid4().hex
            extra=['--lanechange.duration','5'] if mode=='avoid' else []
            traci.start([str(binary),'-c',str(config),'--seed',str(seed),*extra],label=label)
            self.connection=traci.getConnection(label)
            c=self.connection
            c.route.add('road',['highway_0','highway_1'])
            c.vehicletype.copy('DEFAULT_VEHTYPE','lidar_car')
            c.vehicletype.setLength('lidar_car',5.)
            c.vehicletype.setWidth('lidar_car',2.)
            c.vehicletype.setAccel('lidar_car',2.)
            c.vehicletype.setDecel('lidar_car',4.5)
            c.vehicletype.setImperfection('lidar_car',0.)
            if mode=='follow':
                c.vehicle.add('lead','road',typeID='lidar_car',departPos=str(20+gap+5),departSpeed='4',departLane='0')
            c.vehicle.add('ego','road',typeID='lidar_car',departPos='20',departSpeed=str(speed),departLane='0')
            c.simulationStep()
            expected={'ego','lead'} if mode=='follow' else {'ego'}
            if set(c.vehicle.getIDList())!=expected: raise RuntimeError('Vehicles failed to depart')
            for vid in expected:
                # Disable discretionary lane changes while retaining collision avoidance.
                c.vehicle.setLaneChangeMode(vid,512)
            self.initial_x=c.vehicle.getPosition('ego')[0]
        except BaseException:
            self.close(); raise

    def snapshot(self):
        c=self.connection
        return {'time':c.simulation.getTime(),'vehicles':{v:{'x':c.vehicle.getPosition(v)[0],
            'y':c.vehicle.getPosition(v)[1],'speed':c.vehicle.getSpeed(v),'angle':c.vehicle.getAngle(v)}
            for v in c.vehicle.getIDList()},'collisions':list(c.simulation.getCollidingVehiclesIDList())}

    def step(self,speed,lane_request=None):
        c=self.connection
        if lane_request is not None:
            if self.mode!='avoid' or lane_request not in (0,1): raise ValueError('Invalid lane request')
            c.vehicle.changeLane('ego',lane_request,5.)
        c.vehicle.setSpeed('ego',float(speed))
        if self.mode=='follow': c.vehicle.setSpeed('lead',4. if c.simulation.getTime()<8. else 0.)
        c.simulationStep()
        return self.snapshot()

    def close(self):
        c,self.connection=self.connection,None
        if c is not None: c.close()
