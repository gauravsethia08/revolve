import json
import h5py
import pandas as pd

def h5_to_csv(h5_file, csv_file):
    with h5py.File(h5_file, 'r') as h5f:
        # key = list(h5f.keys())
        # h5f_data = h5f[key[0]]
        # if 'current_target' in h5f_data:
        #     print(h5f_data['current_target'][:])


        for replay in list(h5f.keys()):
            demo_data = h5f[replay]
            current_target = demo_data['current_target'][:]
            # print(current_target)
            qpos = demo_data['qpos'][:]
            # print(qpos)
            action = demo_data['action'][:]
            # print(action)
            data = {
                'current_target': current_target.tolist(),
                'qpos': qpos.tolist(),
                'action': action.tolist()
            }
            with open(f"demo_{replay}.json", 'w') as json_file:
                json.dump(data, json_file)
            # df = pd.DataFrame(data)
            # df.to_csv(f"demo_{replay}.csv", mode='a', header=not pd.io.common.file_exists(csv_file), index=False)
           
            

        # data = {key: h5f[key][:] for key in h5f.keys() if isinstance(h5f[key], h5py.Dataset)}
        # df = pd.DataFrame(data)
        # df.to_csv(csv_file, index=False)

# Example usage
h5_file = 'real_data.h5'
csv_file = 'output.csv'
h5_to_csv(h5_file, csv_file)