from pylab import *


for solver in ["ipopt","blocksqp"]:

    logfile = eval(file('/home/tim/Dropbox/EigenDocumenten/Doctoraat/MotionPlanning/omg-tools/examples/testlog_%s.txt' % solver,'r').read())

    overhead = np.array([ sum(d[k] for k in d.keys() if k.startswith('t_wall') and 'mainloop' not in k)  for d in logfile])
    mainloop =  np.array([d['t_wall_mainloop']  for d in logfile])
    iter_count =  np.array([d['iter_count']  for d in logfile])

    figure(1)
    semilogy(mainloop,label=solver)
    semilogy(mainloop-overhead,'--',label=solver)
    title('mainloop')
    legend()

    figure(2)
    plot(iter_count,label=solver)
    title('it')
    legend()

    figure(3)
    semilogy((mainloop-overhead)/iter_count,label=solver)
    title('time per nlp it')
    legend()

    if solver == "blocksqp":
        figure(4)
        plot([d['nRestPhaseCalls']  for d in logfile],label=solver)
        title('resto')
        legend()

        figure(5)
        plot([d['qpItTotal']  for d in logfile],label=solver)
        title('qps solved')
        legend()

show()
