"""Batched two-car TraCI adapter; SUMO alone owns both vehicles' motion."""
import math
import uuid
import traci


class FleetTraffic:
    vehicle_ids=('ego','peer')

    def __init__(self,binary,config,seed=42,speed=6.,rear_gap=6.):
        self.connection=None
        try:
            label='fleet-'+uuid.uuid4().hex
            traci.start([str(binary),'-c',str(config),'--seed',str(seed),
                '--lanechange.duration','5'],label=label)
            self.connection=traci.getConnection(label);c=self.connection
            c.route.add('fleet_road',['highway_0','highway_1'])
            c.vehicletype.copy('DEFAULT_VEHTYPE','fleet_car')
            for method,value in [('setLength',5.),('setWidth',2.),('setAccel',2.),('setDecel',4.5),('setImperfection',0.)]:
                getattr(c.vehicletype,method)('fleet_car',value)
            for vid,x,lane in [('ego',40.,0),('peer',40.-rear_gap,1)]:
                c.vehicle.add(vid,'fleet_road',typeID='fleet_car',departPos=str(x),departSpeed=str(speed),departLane=str(lane))
            c.simulationStep()
            if set(c.vehicle.getIDList())!=set(self.vehicle_ids): raise RuntimeError('Fleet departure failed')
            for vid in self.vehicle_ids:
                c.vehicle.setLaneChangeMode(vid,512)
                c.vehicle.subscribe(vid,[traci.constants.VAR_POSITION,traci.constants.VAR_SPEED,traci.constants.VAR_ANGLE])
        except BaseException:
            self.close();raise

    def snapshot(self):
        c=self.connection;active=set(c.vehicle.getIDList());vehicles={}
        if not set(self.vehicle_ids)<=active: raise RuntimeError('Controlled vehicle left fixture')
        for vid in self.vehicle_ids:
            s=c.vehicle.getSubscriptionResults(vid)
            vehicles[vid]=dict(x=s[traci.constants.VAR_POSITION][0],y=s[traci.constants.VAR_POSITION][1],
                speed=s[traci.constants.VAR_SPEED],angle=s[traci.constants.VAR_ANGLE])
        return dict(time=c.simulation.getTime(),vehicles=vehicles,collisions=list(c.simulation.getCollidingVehiclesIDList()))

    def step(self,actions):
        if set(actions)!=set(self.vehicle_ids): raise ValueError('Exactly one action per vehicle required')
        # Validate the entire batch before any mutation.
        for speed,lane in actions.values():
            if not math.isfinite(speed) or speed<0 or lane not in (None,0,1): raise ValueError('Invalid action')
        c=self.connection
        for vid,(speed,lane) in actions.items():
            if lane is not None: c.vehicle.changeLane(vid,lane,5.)
            c.vehicle.setSpeed(vid,float(speed))
        c.simulationStep()
        return self.snapshot()

    def close(self):
        c,self.connection=self.connection,None
        if c is not None:c.close()
