# Calibrazione Loco Positioning System

Codice di calibrazione automatica del Loco Positioning System con 8 ancore tramite ottimizzazione.

Il repository è strutturato nel seguente modo:
- Il file log_util.py semplifica l'utilizzo dell'API di logging dei Crazyflie.
- Il file optimization.py è l'implemetazione della procedura di ottimizzazione che utilizza la libreria SciPy.
- Il file write_to_anchors.py implementa la scrittura delle coordinate sulle ancore.
- il file calibration.py effettua la procedura di calibrazione e la scrittura automatica delle coordinate sulle varie ancore.