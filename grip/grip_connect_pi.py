from networktables import NetworkTables

NetworkTables.initialize(server="roborio-2605-frc.local")

testTable = NetworkTables.getTable('test-table')

while True:
    testTable.putNumber('testNumber', 1235)
    print(testTable.getNumber('testNumber'))
