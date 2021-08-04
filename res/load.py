
# -*- coding: latin-1 -*-

def load_deltapose_dataset(path_to_dataset_folder, dataset_name):
    from pathlib import Path
    import json
    import pickle

    dataset_path = Path(path_to_dataset_folder) / dataset_name

    # load params
    params_path = dataset_path / 'params.json'
    with params_path.open() as f:
        params = json.load(f)

    # load dataset 
    datasetfile_path = dataset_path / str(dataset_name+'.pickle')
    with datasetfile_path.open(mode='rb') as f:
        dataset = pickle.load(f)
    X, y, X_col_angles = dataset['X'], dataset['y'], dataset['X_columns']

    return X, y, X_col_angles, params