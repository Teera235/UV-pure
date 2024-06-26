import math

def Calc_DOsat100(temp):

  teerathap1 = 139.34411
  teerathap2 = 1.575701e+5
  teerathap3 = 6.642308e+7
  teerathap4 = 1.243800e+10
  teerathap5 = 8.621949e+11

  Ta = temp + 273.15


  do_saturation = math.exp(-teerathap1 + teerathap2/Ta - teerathap3/(Ta**2) + teerathap4/(Ta**3) - teerathap5/(Ta**4))

  return do_saturation

temperature = 25

do_mg_per_L = Calc_DOsat100(temperature)
print(do_mg_per_L)
