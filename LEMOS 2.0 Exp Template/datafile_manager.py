import pandas as pd
import os

def read_and_save(filename):
    global datafile_filename
    global fl_df, od_df
    global fl_last_row, od_last_row

    datafile_filename = filename

    # how many columns of data there are; needed to skip the blank rows
    num_data_cols = 34

    fl_df = pd.read_csv(filename, encoding='unicode_escape')
    second_table_loc = fl_df.Time[fl_df.Time == 'Time'].index

    # end the first table when reaching the next set of column headers
    fl_df = fl_df.iloc[0:second_table_loc[0]]
    #fl_df.dropna(how="any", thresh=num_data_cols, inplace=True)
    fl_df.dropna(thresh=num_data_cols, inplace=True)

    # get latest row data as a dictionary
    fl_last_row = fl_df.tail(-1).to_dict()

    """**Find Second Table**"""

    od_df = pd.read_csv(filename, encoding='unicode_escape', header=second_table_loc[0]+1)
    # od_df.dropna(how="any", thresh=num_data_cols, inplace=True)
    od_df.dropna(thresh=num_data_cols, inplace=True)

    # get latest row data as a dictionary
    od_last_row = od_df.tail(-1).to_dict()

    """**Save to CSV**"""

    fl_df.to_csv('Datafile/fl.csv', index=False)
    od_df.to_csv('Datafile/od.csv', index=False)

def get_fl_latest():
    return fl_last_row

def get_od_latest():
    return od_last_row

def remove():
    os.remove(datafile_filename)