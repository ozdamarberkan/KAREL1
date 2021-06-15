import socket
import string
import pymongo
import threading
import time as t
import logging
import numpy as np
import pickle

from pymongo import MongoClient
from HDE_LSTM import HDE_LSTM
from keras.models import load_model
from keras.models import model_from_json

def json_formatting(m):
    lines = m.split("\n")
    data_dict = {}

    for line in lines:
        if "=" in line:
            key, value = line.split("=")
            if key == "time":
                elements = value.split(" ")
                for element in elements:
                    if ":" in element:
                        data_dict[key] = element
            else:
                data_dict[key] = value
    return data_dict

def prep_data(data):
    samples = []
    for data_point in data:
        sample = []
        for key in data_point:
            if key != "time" and key != "_id" and key != "AP":
                sample.append(float(data_point[key]))
        samples.append(sample)

    return np.array(samples)

def proper_receive(db, s):
    # receiver data
    client, addr = s.accept()

    lines = []
    while True:
        content = client.recv(32)
 
        if len(content) == 0:
            break
        else:
            lines.append(content.decode("UTF-8"))

    # transform into json
    message = "".join(lines)
    to_upload = json_formatting(message)
    print(to_upload)

    # add to corresponding AP collection
    #ap_no = to_upload["AP"]
    #collection = assign_ap(data_database, " ")
    #collection.insert_one(to_upload)
    
    response_proto = 'HTTP/1.1'
    response_status = '200'
    response_status_text = 'OK' # this can be random 

    # sending all this stuff
    client.send(str.encode('%s %s %s' % (response_proto, response_status, response_status_text)))
    client.close() 

    return to_upload

def assign_ap(db, ap_no):
    
    if ap_no == "1":
        collection = db.AP1
    elif ap_no == "2":
        collection = db.AP2
    elif ap_no == "3":
        collection = db.AP3
    elif ap_no == "4":
        collection = db.AP4
    elif ap_no == "5":
        collection = db.AP5
    else:
        collection = db.ExtraClass1

    return collection

# For MongoDB Authentication
client_mongo = MongoClient("mongodb+srv://Karel_Admin:KarelNetwork@karelnetwork.90pmb.mongodb.net/<dbname>?retryWrites=true&w=majority")
data_database = client_mongo.get_database("Data")

# Socket Parameters
s = socket.socket()        
s.bind(("139.179.32.248", 8001))
s.listen(0)                 

while True:
    # receiving and transforming into json and sends to database   
    data_point = proper_receive(data_database, s)
