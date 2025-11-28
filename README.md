GNSS_NEMA_Parser.py

Parse NMEA sentences from a text file, validate checksums, extract fixes (GGA/RMC),
compute summary statistics (fix count, time range, sat counts, HDOP, altitude min/max),
and compute total distance traveled using haversine formula.

Usage:
    python GNSS_NEMA_Parser.py path/to/nmea.txt --out fixes.csv

Outputs summary to stdout and optionally saves a CSV of parsed fixes.

<u>Example</u>
python .\GNSS_NEMA_Parser.py .\GPS_OLDANT_150MMCOAX.txt                                                                     
NMEA parsing summary:
  Total parsed sentences: 110
  Valid position fixes:   110
  Time range:             2025-11-11T11:33:48+00:00 -> 2025-11-11T11:34:42+00:00
  In view Satellites (max/avg, samples): 16.00/10.90 (count 330)
  Active Satellites (min/avg/max): 7/7.00/7
  HDOP (min/avg/max):       1.87/2.72/5.87
  Altitude (min/avg/max m): 836.00/836.20/836.60
  Total distance (meters):  36.00
  In-view SNR (min/avg/max dBHz):   11.00/23.38/36.00 (samples 495)
  Active SNR (min/avg/max dBHz):    12.00/25.17/36.00 (samples 378)
