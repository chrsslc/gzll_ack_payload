
from flask import Flask
import MySQLdb
localDB = MySQLdb.connect("localhost","root","raspberry","testQueue")

#cur = localDB.cursor(MySQLdb.cursors.DictCursor)
cur = localDB.cursor(MySQLdb.cursors.Cursor)


app = Flask(__name__)




@app.route('/')
def index():
    cur.execute("Select readings from BufferQueue;")
    #conn = mysql.connect()
    #cursor = conn.cursor()
    #cursor.execute("select VALUE_F from TEMP_DATA LIMIT 1")
    #data = cursor.fetchall()
    #return data
    data = cur.fetchall()
#Added by Kevin
    teststring2 = ''
    print(data)

#    for key, value in data[0]:
#	teststring2 = teststring2 + ', ' + value

    for index, tuple in enumerate(data):
	teststring2 = teststring2 + ', ' +  str(tuple[0])	
#teststring2 = str(tuple[0]) + ': ' + str(tuple[1]) + '. '
#    for row in data:
#	teststring2 = teststring2 + ''.join(row[1])
    return(teststring2)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')

