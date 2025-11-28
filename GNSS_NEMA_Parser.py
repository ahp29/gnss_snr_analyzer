from __future__ import annotations
import argparse
import csv
import datetime
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

#!/usr/bin/env python3
"""
GNSS_NEMA_Parser.py

Parse NMEA sentences from a text file, validate checksums, extract fixes (GGA/RMC),
compute summary statistics (fix count, time range, sat counts, HDOP, altitude min/max),
and compute total distance traveled using haversine formula.

Usage:
    python GNSS_NEMA_Parser.py path/to/nmea.txt --out fixes.csv

Outputs summary to stdout and optionally saves a CSV of parsed fixes.
"""


# Helper functions

def checksum_ok(sentence: str) -> bool:
    """
    Validate NMEA sentence checksum.
    Sentence may start with '$' and contain '*hh' checksum at the end.
    """
    sentence = sentence.strip()
    if not sentence.startswith('$'):
        return False
    if '*' not in sentence:
        return False
    try:
        body, cs_hex = sentence[1:].split('*', 1)
    except ValueError:
        return False
    cs_calc = 0
    for ch in body:
        cs_calc ^= ord(ch)
    try:
        cs = int(cs_hex[:2], 16)
    except ValueError:
        return False
    return cs == cs_calc

def parse_lat_lon(value: str, hemi: str) -> Optional[float]:
    """Convert NMEA lat/lon ddmm.mmmm (or dddmm.mmmm) and hemisphere to decimal degrees."""
    if not value or not hemi:
        return None
    try:
        if '.' not in value:
            # fallback, treat as degrees
            deg = float(value)
            return deg if hemi in ('N', 'E') else -deg
        # split degrees and minutes
        dot = value.index('.')
        # minutes start two digits before dot for lat (ddmm) and three for lon (dddmm)
        # heuristic: if dot index > 4 then degrees is first 3 digits (lon), else 2 digits (lat)
        if dot > 4:
            deg_len = 3
        else:
            deg_len = 2
        degrees = float(value[:deg_len])
        minutes = float(value[deg_len:])
        dec = degrees + minutes / 60.0
        if hemi in ('S', 'W'):
            dec = -dec
        return dec
    except Exception:
        return None

def parse_time_utc(timestr: str) -> Optional[datetime.time]:
    """Parse hhmmss[.sss] into time object."""
    if not timestr:
        return None
    try:
        hh = int(timestr[0:2])
        mm = int(timestr[2:4])
        ss = int(timestr[4:6])
        micro = 0
        if '.' in timestr:
            frac = timestr.split('.', 1)[1]
            micro = int((frac + '000000')[:6])
        return datetime.time(hh, mm, ss, micro)
    except Exception:
        return None

def parse_date_utc(datestr: str) -> Optional[datetime.date]:
    """Parse ddmmyy into date object (assume 2000-2099 for yy 00-99 where <70 -> 2000s)."""
    if not datestr or len(datestr) < 6:
        return None
    try:
        day = int(datestr[0:2])
        month = int(datestr[2:4])
        year = int(datestr[4:6])
        # heuristic: if year >= 70 -> 1900s else 2000s (common for GPS NMEA)
        year += 1900 if year >= 70 else 2000
        return datetime.date(year, month, day)
    except Exception:
        return None

def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return distance in meters between two lat/lon pairs."""
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# NMEA sentence parsers

def parse_gga(fields: List[str]) -> Dict:
    """
    GGA - Global Positioning System Fix Data
    Fields:
    0: $GPGGA
    1: UTC time hhmmss.sss
    2: lat ddmm.mmmm
    3: N/S
    4: lon dddmm.mmmm
    5: E/W
    6: fix quality (0=invalid,1=GPS fix,2=DGPS, etc)
    7: num satellites
    8: HDOP
    9: altitude (meters)
    10: 'M' unit
    11: geoid separation
    12: 'M'
    13: DGPS age
    14: DGPS station id
    """
    out = {}
    try:
        out['type'] = 'GGA'
        out['utc_time'] = parse_time_utc(fields[1]) if len(fields) > 1 else None
        # require at least fields[2] and fields[3]
        out['lat'] = parse_lat_lon(fields[2], fields[3]) if len(fields) > 3 else None
        out['lon'] = parse_lat_lon(fields[4], fields[5]) if len(fields) > 5 else None
        out['fix_quality'] = int(fields[6]) if len(fields) > 6 and fields[6].isdigit() else None
        out['num_sats'] = int(fields[7]) if len(fields) > 7 and fields[7].isdigit() else None
        out['hdop'] = float(fields[8]) if len(fields) > 8 and fields[8] else None
        out['altitude'] = float(fields[9]) if len(fields) > 9 and fields[9] else None
    except Exception:
        pass
    return out

def parse_rmc(fields: List[str]) -> Dict:
    """
    RMC - Recommended Minimum Navigation Information
    Fields:
    0: $GPRMC
    1: UTC time hhmmss.sss
    2: status A=active,V=void
    3: lat ddmm.mmmm
    4: N/S
    5: lon dddmm.mmmm
    6: E/W
    7: speed over ground (knots)
    8: track angle (degrees)
    9: date ddmmyy
    10: magnetic variation
    11: E/W
    """
    out = {}
    try:
        out['type'] = 'RMC'
        out['utc_time'] = parse_time_utc(fields[1]) if len(fields) > 1 else None
        out['status'] = fields[2] if len(fields) > 2 else None
        # require at least fields[3] and fields[4]
        out['lat'] = parse_lat_lon(fields[3], fields[4]) if len(fields) > 4 else None
        out['lon'] = parse_lat_lon(fields[5], fields[6]) if len(fields) > 6 else None
        out['speed_knots'] = float(fields[7]) if len(fields) > 7 and fields[7] else None
        out['course'] = float(fields[8]) if len(fields) > 8 and fields[8] else None
        out['date'] = parse_date_utc(fields[9]) if len(fields) > 9 else None
    except Exception:
        pass
    return out

def parse_gsa(fields: List[str]) -> Dict:
    """
    GSA - GNSS DOP and Active Satellites
    Active satellite PRNs are in fields 2..13 (up to 12 PRNs).
    Returns dict with 'type':'GSA' and list 'active_prns' (strings).
    """
    out = {'type': 'GSA', 'active_prns': []}
    try:
        # active PRNs are typically fields index 2..13
        prn_fields = fields[2:14]
        out['active_prns'] = [p for p in prn_fields if p]
    except Exception:
        pass
    return out

def parse_gsv(fields: List[str]) -> Dict:
    """
    GSV - Satellites in view
    Returns dict with:
      - 'type':'GSV'
      - 'sat_in_view' (int) if present
      - 'sats': list of (prn_str, snr_int_or_None) tuples parsed from groups
    """
    out = {'type': 'GSV', 'sats': [], 'sat_in_view': None}
    try:
        if len(fields) > 3 and fields[3]:
            try:
                out['sat_in_view'] = int(fields[3])
            except ValueError:
                out['sat_in_view'] = None
        idx = 4
        while idx + 3 < len(fields):
            prn = fields[idx]
            # elevation = fields[idx+1]; azimuth = fields[idx+2]
            snr_field = fields[idx + 3]
            snr_val = None
            if snr_field:
                try:
                    snr_val = int(snr_field)
                except ValueError:
                    snr_val = None
            out['sats'].append((prn, snr_val))
            idx += 4
    except Exception:
        pass
    return out

# Main processing logic

def process_file(path: str) -> Tuple[List[Dict], Dict]:
    """
    Read NMEA sentences from file, parse GGA and RMC, assemble fix records.
    Returns list of fixes and summary dictionary.
    """
    fixes: List[Dict] = []
    last_date: Optional[datetime.date] = None
    snr_values: List[int] = []  # legacy, will not be used
    in_view_snr_values: List[int] = []     # SNR samples from GSV (all in-view sats)
    active_snr_values: List[int] = []      # SNR samples for active sats (from GSA -> matched to latest GSV)
    prn_snr_map: Dict[str, int] = {}       # latest seen SNR per PRN (from GSV sentences)
    sat_in_view_values: List[int] = []     # collect satellites-in-view counts from GSV sentences

    with open(path, 'r', encoding='utf-8', errors='ignore') as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line:
                continue
            if not checksum_ok(line):
                continue
            try:
                body = line[1:line.find('*')]
            except Exception:
                continue
            fields = body.split(',')
            talk_id = fields[0] if fields else ''
            parsed = {}
            if 'GGA' in talk_id:
                parsed = parse_gga(fields)
            elif 'RMC' in talk_id:
                parsed = parse_rmc(fields)
            elif 'GSV' in talk_id:
                parsed = parse_gsv(fields)
                # accumulate per-PRN latest SNR and in-view SNR samples
                for prn, snr in parsed.get('sats', []):
                    if snr is not None:
                        prn_snr_map[prn] = snr
                        in_view_snr_values.append(snr)
                if parsed.get('sat_in_view') is not None:
                    sat_in_view_values.append(parsed['sat_in_view'])
                # do not create a position fix for GSV
                continue
            elif 'GSA' in talk_id:
                parsed = parse_gsa(fields)
                # for active PRNs, collect SNRs from latest prn_snr_map (if known)
                for prn in parsed.get('active_prns', []):
                    if prn in prn_snr_map:
                        active_snr_values.append(prn_snr_map[prn])
                # do not create a position fix for GSA
                continue
            else:
                continue

            # Combine parsed into a fix record
            # Use utc datetime when possible: prefer date from RMC, else carry last known date
            utc_time = parsed.get('utc_time')
            date = parsed.get('date')
            if date:
                last_date = date
            if utc_time and last_date:
                dt = datetime.datetime.combine(last_date, utc_time, tzinfo=datetime.timezone.utc)
            elif utc_time and parsed.get('type') == 'RMC' and parsed.get('date'):
                dt = datetime.datetime.combine(parsed['date'], utc_time, tzinfo=datetime.timezone.utc)
            elif utc_time:
                # no known date; use a placeholder date 1970-01-01 but keep ordering by file
                dt = datetime.datetime.combine(datetime.date(1970,1,1), utc_time, tzinfo=datetime.timezone.utc)
            else:
                dt = None

            # merge into fix: if previous fix has lat/lon we can fill missing fields
            fix = {
                'datetime': dt,
                'lat': parsed.get('lat'),
                'lon': parsed.get('lon'),
                'altitude': parsed.get('altitude'),
                'num_sats': parsed.get('num_sats'),
                'hdop': parsed.get('hdop'),
                'speed_knots': parsed.get('speed_knots'),
                'source': parsed.get('type'),
                'line_no': lineno,
                'raw': line,
            }

            # Try to merge with previous fix if lat/lon missing: but keep simple - append as separate fixes
            fixes.append(fix)

    # Post-process: create consolidated fixes where lat/lon present
    consolidated: List[Dict] = []
    last_known: Dict = {}
    for f in fixes:
        # carry forward last known lat/lon if missing
        lat = f['lat'] if f['lat'] is not None else last_known.get('lat')
        lon = f['lon'] if f['lon'] is not None else last_known.get('lon')
        if lat is not None and lon is not None:
            rec = {
                'datetime': f['datetime'],
                'lat': lat,
                'lon': lon,
                'altitude': f['altitude'] if f['altitude'] is not None else last_known.get('altitude'),
                'num_sats': f['num_sats'] if f['num_sats'] is not None else last_known.get('num_sats'),
                'hdop': f['hdop'] if f['hdop'] is not None else last_known.get('hdop'),
                'speed_knots': f['speed_knots'] if f['speed_knots'] is not None else last_known.get('speed_knots'),
                'source': f['source'],
                'line_no': f['line_no'],
                'raw': f['raw'],
            }
            consolidated.append(rec)
            last_known.update(rec)
        else:
            # skip fixes without position
            continue

    # Sort consolidated fixes by datetime if available else by original order
    consolidated.sort(key=lambda x: (x['datetime'] or datetime.datetime.min))

    # Compute summary stats
    summary = {}
    summary['total_sentences'] = len(fixes)
    summary['valid_fixes'] = len(consolidated)
    # time range
    times = [f['datetime'] for f in consolidated if f['datetime'] is not None]
    if times:
        summary['start_time'] = min(times).isoformat()
        summary['end_time'] = max(times).isoformat()
    else:
        summary['start_time'] = None
        summary['end_time'] = None
    # satellites and hdop
    sats = [f['num_sats'] for f in consolidated if f['num_sats'] is not None]
    hdops = [f['hdop'] for f in consolidated if f['hdop'] is not None]
    alts = [f['altitude'] for f in consolidated if f['altitude'] is not None]
    summary['min_sats'] = min(sats) if sats else None
    summary['max_sats'] = max(sats) if sats else None
    summary['avg_sats'] = (sum(sats)/len(sats)) if sats else None
    summary['min_hdop'] = min(hdops) if hdops else None
    summary['max_hdop'] = max(hdops) if hdops else None
    summary['avg_hdop'] = (sum(hdops)/len(hdops)) if hdops else None
    summary['min_alt'] = min(alts) if alts else None
    summary['max_alt'] = max(alts) if alts else None
    summary['avg_alt'] = (sum(alts)/len(alts)) if alts else None

    # SNR summary (from GSV sentences -> in-view)
    summary['in_view_snr_count'] = len(in_view_snr_values)
    summary['in_view_min_snr'] = min(in_view_snr_values) if in_view_snr_values else None
    summary['in_view_max_snr'] = max(in_view_snr_values) if in_view_snr_values else None
    summary['in_view_avg_snr'] = (sum(in_view_snr_values)/len(in_view_snr_values)) if in_view_snr_values else None

    # Active satellites SNR summary (matched via GSA -> GSV mapping)
    summary['active_snr_count'] = len(active_snr_values)
    summary['active_min_snr'] = min(active_snr_values) if active_snr_values else None
    summary['active_max_snr'] = max(active_snr_values) if active_snr_values else None
    summary['active_avg_snr'] = (sum(active_snr_values)/len(active_snr_values)) if active_snr_values else None

    # Satellites-in-view summary (from GSV sentences)
    summary['sat_in_view_count'] = len(sat_in_view_values)
    summary['max_sat_in_view'] = max(sat_in_view_values) if sat_in_view_values else None
    summary['avg_sat_in_view'] = (sum(sat_in_view_values)/len(sat_in_view_values)) if sat_in_view_values else None

    # compute total distance
    total_m = 0.0
    prev = None
    for f in consolidated:
        if prev:
            total_m += haversine(prev['lat'], prev['lon'], f['lat'], f['lon'])
        prev = f
    summary['total_distance_m'] = total_m

    return consolidated, summary

def save_csv(path: str, fixes: List[Dict]) -> None:
    fieldnames = ['datetime', 'lat', 'lon', 'altitude', 'num_sats', 'hdop', 'speed_knots', 'source', 'line_no', 'raw']
    with open(path, 'w', newline='', encoding='utf-8') as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for f in fixes:
            row = {k: f.get(k) for k in fieldnames}
            # format datetime ISO or empty
            row['datetime'] = row['datetime'].isoformat() if row['datetime'] else ''
            writer.writerow(row)
def print_summary(summary: Dict) -> None:
    print("NMEA parsing summary:")
    print(f"  Total parsed sentences: {summary.get('total_sentences')}")
    print(f"  Valid position fixes:   {summary.get('valid_fixes')}")
    st = summary.get('start_time') or 'N/A'
    et = summary.get('end_time') or 'N/A'
    print(f"  Time range:             {st} -> {et}")
    # Satellites-in-view and Active Satellites (from GSV/GSA)
    print(f"  In view Satellites (max/avg, samples): {_fmt(summary.get('max_sat_in_view'))}/{_fmt(summary.get('avg_sat_in_view'))} (count {summary.get('sat_in_view_count')})")
    print(f"  Active Satellites (min/avg/max): {summary.get('min_sats')}/{_fmt(summary.get('avg_sats'))}/{summary.get('max_sats')}")
    print(f"  HDOP (min/avg/max):       {_fmt(summary.get('min_hdop'))}/{_fmt(summary.get('avg_hdop'))}/{_fmt(summary.get('max_hdop'))}")
    print(f"  Altitude (min/avg/max m): {_fmt(summary.get('min_alt'))}/{_fmt(summary.get('avg_alt'))}/{_fmt(summary.get('max_alt'))}")
    print(f"  Total distance (meters):  {summary.get('total_distance_m'):.2f}")
    # SNR summaries separated
    print(f"  In-view SNR (min/avg/max dBHz):   {_fmt(summary.get('in_view_min_snr'))}/{_fmt(summary.get('in_view_avg_snr'))}/{_fmt(summary.get('in_view_max_snr'))} (samples {summary.get('in_view_snr_count')})")
    print(f"  Active SNR (min/avg/max dBHz):    {_fmt(summary.get('active_min_snr'))}/{_fmt(summary.get('active_avg_snr'))}/{_fmt(summary.get('active_max_snr'))} (samples {summary.get('active_snr_count')})")

def _fmt(v):
    if v is None:
        return 'N/A'
    if isinstance(v, (int, float)):
        return f"{v:.2f}"
    return str(v)

def main():
    parser = argparse.ArgumentParser(description="Parse NMEA sentences from a file and produce summary + CSV.")
    parser.add_argument('input', help="Path to NMEA text file")
    parser.add_argument('--out', '-o', help="Optional CSV output path for parsed fixes")
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"Input file not found: {args.input}", file=sys.stderr)
        sys.exit(1)

    fixes, summary = process_file(args.input)
    print_summary(summary)
    if args.out:
        save_csv(args.out, fixes)
        print(f"CSV saved to: {args.out}")

if __name__ == '__main__':
    main()
