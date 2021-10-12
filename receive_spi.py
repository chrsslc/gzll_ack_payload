import spidev
import MySQLdb
localDB = MySQLdb.connect("localhost","root","raspberry","testQueue")

cur = localDB.cursor(MySQLdb.cursors.DictCursor)



def split(word):
    return [char for char in word]

spi = spidev.SpiDev() 

spi.open(0, 1) 

spi.max_speed_hz = 500000 

spi.mode = 0 
#recv = spi.xfer2(bytearray("hello, world", "utf-8")) 
msg_tx = ["thisisatest12XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"]
#encoded_string = msg_tx.encode()


#byte_array_msg = bytearray(encoded_string)

testString = '1, ' * 63
testString = '[' +  testString + '1]'
#print(testString)

#print(byte_array_msg)
#recv = spi.xfer2([1, 2, 3, 4, 5, 6, 7 ,8, 9, 10, 11, 12])
#testArray = split(msg_tx)
testList = range(32, 96)
#testList = range(85, 90)
#print(testList)

# Convert String list to ascii values
# using loop + ord()
res = []
for ele in msg_tx:
    res.extend(ord(num) for num in ele)

recv = spi.xfer2(res)
#recv = spi.xfer2(byte_array_msg) 
#recv = spi.xfer2([1,250,0])
print("Received %s" % bytearray(recv))
teststring = str(bytearray(recv))
if teststring == 'FROMNRF2XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX':
	teststring = teststring	
else:
	#print("Recieved %s" % recv)
	#print(teststring)


	query = "INSERT INTO BufferQueue VALUES ('%s');" % (teststring)

	cur.execute(query)
	localDB.commit()
cur.close()
localDB.close()



spi.close()

