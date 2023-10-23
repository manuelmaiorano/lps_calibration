import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from lpslib.lopoanchor import LoPoAnchor

N_ANCHORS = 8

URI = 'radio://0/80/2M/E7E7E7E703'

coords = [[ 0.      ,    0.      ,    0.    ,    ],
 [-0.42417068 , 3.20219564 , 2.25247497],
 [ 2.84341933 , 3.86533445 , 0.0904174 ],
 [ 4.11085898 ,-0.56572359 , 2.19364089],
 [-0.33760282 ,-0.25433495 , 2.08982492],
 [ 0.         , 3.27411219 , 0.        ],
 [ 2.9691431  , 4.17399099 , 2.34229375],
 [ 3.85913983 ,-0.36607976 , 0.        ]]


def write_to_anchors(coords):
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        anchors = LoPoAnchor(scf.cf)
        for _ in range(3):
            for i in range(N_ANCHORS):
                anchors.set_position(i, coords[i][:])


if __name__ == "__main__":
    write_to_anchors(coords)