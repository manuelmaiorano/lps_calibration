from cflib.crazyflie.log import LogConfig

battery_log = [('pm.batteryLevel', 'uint8_t'), ('pm.vbat', 'float')]
kalman_log = [('kalman.varPX', 'float'), ('kalman.varPY', 'float'), ('kalman.varPZ', 'float')]

def log_callback(name, timestamp, data, logconf):
    print("-"*50)
    print(name)
    print(data)
    print("-"*50)

def add_log(scf, name, vars, period_in_ms=100, cb=log_callback):
    logconf = LogConfig(name=name, period_in_ms=period_in_ms)
    for var in vars:
        logconf.add_variable(var[0],var[1])
    scf.cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(lambda timestamp, data, logconf: cb(name, timestamp, data, logconf))
    logconf.start()