from flask import Flask, render_template, g
import os
import requests
import json
import sqlite3
from datetime import datetime

app = Flask(__name__)
app.config['DATABASE_notifications'] = os.path.join(os.getcwd(), 'notifications.db')

def get_db_notifications():
    db = getattr(g, '_database_notifications', None)
    if db is None:
        db = g._database_notifications = sqlite3.connect(app.config['DATABASE_notifications'])
        db.row_factory = sqlite3.Row
    return db

def init_db_notifications():
    with app.app_context():
        db = get_db_notifications()
        cursor = db.cursor()
        cursor.execute('CREATE TABLE IF NOT EXISTS notifications (id INTEGER PRIMARY KEY AUTOINCREMENT, alert TEXT, timestamp TEXT)'
                        'CREATE TABLE IF NOT EXISTS SpO2 (id INTEGER PRIMARY KEY AUTOINCREMENT, alert TEXT, timestamp TEXT)'
                         'CREATE TABLE IF NOT EXISTS Heart_Rate (id INTEGER PRIMARY KEY AUTOINCREMENT, alert TEXT, timestamp TEXT)'
                            'CREATE TABLE IF NOT EXISTS BP (id INTEGER PRIMARY KEY AUTOINCREMENT, alert TEXT, timestamp TEXT)'
                             'CREATE TABLE IF NOT EXISTS Temp (id INTEGER PRIMARY KEY AUTOINCREMENT, alert TEXT, timestamp TEXT)')
        db.commit()

@app.route('/')
def start():
    return render_template('index.html')

@app.route('/index')
def index():
    return render_template('index.html')

@app.route('/dep')
def dep():
    return render_template('departments.html')

@app.route('/depData')
def depData():
    return render_template('history.html')

@app.route('/about')
def about():
    return render_template('about.html')

@app.route('/analyse')
def analyse():
    return render_template('analyse.html')

@app.route('/spO2')
def spO2():
    data = 'SpO2'
    return  getData(data, 95, 100, 'SpO2 abnormal')

@app.route('/temperature')
def temperature():
    data = 'Temp'
    return getData(data, 97, 98, 'temp abnormal')

@app.route('/bpsys')
def bps():
    data1 = 'BP_sys'
    return getData(data1,110, 120, 'bp sys abnormal')

@app.route('/bpdia')
def bpd():
    data2 = 'BP_dia'
    return getData(data2, 75, 85, 'dia abnormal')


@app.route('/heartRate')
def heartRate():
    data = 'Heart_Rate'
    return getData(data,60, 100, 'heart attack')

def getData(data, par1, par2, notify):
    url = 'http://127.0.0.1:5089/~/in-cse/in-name/Patient_Monitor/' + data + '/Data?rcn=4'
    headers = {
        'X-M2M-Origin' : 'admin:admin',
        'Content-type' : 'application/json',
        'Accept' : 'application/json'
    }

    response = requests.get(url, headers=headers)
    response = json.loads(response.text)['m2m:cnt']
    j = 0
    result = []
    for i in response:
        value = response['m2m:cin'][j]['con']
        result.append(value)

        if float(value) > par1 or float(value) < par2 :
            react = notify
            current_time = datetime.now()
            db = get_db_notifications()
            cursor = db.cursor()
            cursor.execute('INSERT INTO' + data + '(alert, timestamp) VALUES (?,?)',(react, current_time,))
            cursor.execute('INSERT INTO notifications (alert, timestamp) VALUES (?,?)',(react, current_time,))
            db.commit()
            cursor.close()
                      
        j += 1
    string = f"{data}.html"
    db = get_db_notifications()
    cursor = db.cursor()
    cursor.execute(f"SELECT * FROM {data}")
    notifications = cursor.fetchall()
    print(notifications)
    cursor.close()
    return render_template(string, data = notifications)

if __name__ == '__main__':
    app.run(debug=True)

