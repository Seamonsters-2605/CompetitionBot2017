from networktables import NetworkTables

NetworkTables.initialize(server="roborio-2605-frc.local")

testTable = NetworkTables.getTable('test-table')
testTable.putNumber('testNumber', 1234)
