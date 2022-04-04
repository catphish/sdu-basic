#f = File.open('table.csv', 'w')
values = [[], [], []]
8192.times do |n|
  angle = (Math::PI * 2 * n) / 8192
  v1 = Math.sin(angle)
  v2 = Math.sin(angle + (Math::PI * 2) / 3)
  v3 = Math.sin(angle + (Math::PI * 4) / 3)
  mid = ([v1,v2,v3].min + [v1,v2,v3].max) / 2
  vv1 = (v1 - mid) / 1.732050808 * 2
  vv2 = (v2 - mid) / 1.732050808 * 2
  vv3 = (v3 - mid) / 1.732050808 * 2
  values[0] << vv1
  values[1] << vv2
  values[2] << vv3
#  f.write([vv1,vv2,vv3].join(','))
#  f.write("\n")
end

f = File.open('table.h', 'w')

f.write("static float table1[] = { ")
f.write(values[0].join(", "))
f.write(" };\n")

f.write("static float table2[] = { ")
f.write(values[1].join(", "))
f.write(" };\n")

f.write("static float table3[] = { ")
f.write(values[2].join(", "))
f.write(" };\n")
