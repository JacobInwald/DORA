"""
For converting Labelbox .ndjson v2 exports to .json.
Roboflow only supports Labelbox v1 (deprecated) exports, so this file is technically useless.
"""

def convert(filename):
    with open(filename, 'r') as f_in:
        with open(f'{filename[:-7]}.json', 'w') as f_out:
            f_out.write('[')
            f_out.write(f_in.readline())
            while True:
                line = f_in.readline()
                if not line:
                    break
                f_out.write(',')
                f_out.write(line)
            f_out.write(']')

if __name__ == '__main__':
    convert('../../data/labelbox/Export-Toy_Detection-2_3_2024.ndjson')
