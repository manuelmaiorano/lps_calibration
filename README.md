# Calibrazione Loco Positioning System

Codice di calibrazione automatica del Loco Positioning System con 8 ancore tramite ottimizzazione.

Il repository è strutturato nel seguente modo:
- Il file log_util.py semplifica l'utilizzo dell'API di logging dei Crazyflie.
- Il file optimization.py è l'implemetazione della procedura di ottimizzazione che utilizza la libreria SciPy.
- Il file write_to_anchors.py implementa la scrittura delle coordinate sulle ancore.
- il file calibration.py effettua la procedura di calibrazione e la scrittura automatica delle coordinate sulle varie ancore.

# Utilizzo
Per utilizzare il sistema di calibrazione è necessario modificare il firmware di un drone Crazyflie utilizzando la procedura descritta [qui](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/) compilando il [firmware modificato](https://github.com/manuelmaiorano/crazyflie-firmware). Si assume che il setup sia quello con 8 ancore descritto nella [documentazione](https://www.bitcraze.io/documentation/tutorials/getting-started-with-loco-positioning-system/).

Una volta che si è compilato e caricato il firmware su un drone Crazyflie, si esegue lo script calibration.py avendo cura di inserire l'indirizzo corretto del drone.




