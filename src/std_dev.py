import csv
import numpy as np

values = []

with open('Graph1.txt') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        if line_count == 0:
            # print(f'Column names are {", ".join(row)}')
            name = row[1]
            line_count += 1
        else:
            # print(f'\t{row[0]} time had value of  {row[1]}.')
            line_count += 1
            values.append(float(row[1]))
    # print(values)  
    std_dev = np.std(values)
    print("The {} has a standard deviation of {} calculated from {} data points".format(name, std_dev, (line_count-1)))
    # print(std_dev)
    # print(f'Processed {line_count} lines.')

with open('Graph2.txt') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        if line_count == 0:
            # print(f'Column names are {", ".join(row)}')
            name = row[1]
            line_count += 1
        else:
            # print(f'\t{row[0]} time had value of  {row[1]}.')
            line_count += 1
            values.append(float(row[1]))
    # print(values)  
    std_dev = np.std(values)
    print("The {} has a standard deviation of {} calculated from {} data points".format(name, std_dev, (line_count-1)))
    # print(std_dev)
    # print(f'Processed {line_count} lines.')
