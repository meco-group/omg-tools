kul_blue        = [17./255.,110./255.,138./255.]
kul_red         = [138./255.,31./255.,17./255.]
kul_green       = [17./255.,138./255.,19./255.]
kul_lightblue   = [106./255., 194./255., 238./255.]

color_tmpl  = [kul_blue, kul_red, kul_green, kul_lightblue, kul_blue, kul_red, kul_green, kul_lightblue, kul_blue, kul_red, kul_green, kul_lightblue, kul_blue, kul_red, kul_green, kul_lightblue, kul_blue, kul_red, kul_green, kul_lightblue]

def mixwithwhite(color, perc_white = 80.):
    red     = color[0]
    green   = color[1]
    blue    = color[2]

    red_m   = ((100. - perc_white)*red + perc_white)/100.
    green_m = ((100. - perc_white)*green + perc_white)/100.
    blue_m  = ((100. - perc_white)*blue + perc_white)/100.
    return [red_m, green_m, blue_m]
