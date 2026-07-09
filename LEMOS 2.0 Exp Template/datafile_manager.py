"""
datafile_manager.py
--------------------
Handles reading the CSV file exported by the plate-reader software and
splitting it into the two data tables it contains:
 
    1) "fl" table  -> fluorescence readings
    2) "od" table  -> optical density (absorbance) readings
 
The exported CSV has BOTH tables stacked in a single file. The second
table starts with its own repeated header row (another "Time" column),
so we find that row and use it to split the file into two separate
DataFrames.
 
This module keeps its data (fl_df, od_df, etc.) in module-level globals
so other files can call read_and_save() once and then call 
get_fl_latest()/get_od_latest() to pull the most recent row without
re-reading the file.
"""

import pandas as pd
import os

def read_and_save(filename):
    """
    Read the plate-reader export CSV, split it into the fl/od tables,
    clean up incomplete rows, store the latest row of each table in
    module-level globals, and save cleaned copies to Datafile/fl.csv
    and Datafile/od.csv.
    """
    global datafile_filename
    global fl_df, od_df
    global fl_last_row, od_last_row

    datafile_filename = filename

    # How many non-empty columns a "real" data row must have to be kept.
    # Rows with fewer non-null values than this are considered blank/
    # incomplete (e.g. spacer rows between tables, partially written
    # rows) and get dropped by dropna(thresh=...).
    # NOTE: if the plate-reader export format changes (columns added or
    # removed), this number needs to be updated to match.
    num_data_cols = 34

    # --- First table: fluorescence ("fl") ---
    fl_df = pd.read_csv(filename, encoding='unicode_escape')
    
    # The second table in the file starts with another row whose "Time"
    # column literally contains the text "Time" again. Find where that
    # occurs so we know where table 1 ends and table 2 begins.
    second_table_loc = fl_df.Time[fl_df.Time == 'Time'].index
    fl_df = fl_df.iloc[0:second_table_loc[0]]
    
    # Drop rows that don't have at least num_data_cols non-null values
    # (i.e. incomplete/blank rows).
    fl_df.dropna(thresh=num_data_cols, inplace=True)

    # get latest row data as a dictionary
    fl_last_row = fl_df.tail(-1).to_dict()

    """**Find Second Table**"""
    # --- Second table: optical density ("od") ---
    # Re-read the same file, but this time tell pandas the header row
    # is the one right after the "Time" marker we found above, so the
    # second table gets parsed with its own proper column names.

    od_df = pd.read_csv(filename, encoding='unicode_escape', header=second_table_loc[0]+1)
    od_df.dropna(thresh=num_data_cols, inplace=True)

    # get latest row data as a dictionary
    od_last_row = od_df.tail(-1).to_dict()

    """**Save to CSV**"""

    fl_df.to_csv('Datafile/fl.csv', index=False)
    od_df.to_csv('Datafile/od.csv', index=False)

def get_fl_latest():
    """Return the fluorescence data currently stored from the last read_and_save() call."""
    return fl_last_row

def get_od_latest():
    """Return the optical density data currently stored from the last read_and_save() call."""
    return od_last_row

def remove():
    """
    Delete the raw export file that was last processed by read_and_save().
    This is used so the plate-reader software can export a fresh file
    with the same name next cycle without a "file already exists" issue.
    """
    os.remove(datafile_filename)