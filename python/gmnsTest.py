import osm2gmns as og

net = og.getNetFromFile('test_01.osm', default_lanes=True)
og.consolidateComplexIntersections(net, auto_identify=False,  int_buffer=50.0)
og.outputNetToCSV(net)

