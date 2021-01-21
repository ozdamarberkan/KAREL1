from pymongo import MongoClient
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA

import plotly.express as px
import imblearn
import dill
import numpy as np
import pandas as pd

### DATABASE ACCESS ---------------------------------------------
# Accessing Database
client_mongo = MongoClient("mongodb+srv://Karel_Admin:KarelNetwork@karelnetwork.90pmb.mongodb.net/<dbname>?retryWrites=true&w=majority")
database = client_mongo.get_database('Karel')
rssi_data = database.KarelData

# Retrieving Data
data = []
for document in rssi_data.find():
    data.append(document)

hde_dataframe = pd.DataFrame(data)
hde_dataframe.to_csv("hde_karel_data.csv")

