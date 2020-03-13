%% no inputs
[p0] = definePlantModelSwift021SN018NoAero()

%% no plot
plotflag = 0 ; 
[p1] = definePlantModelSwift021SN018NoAero(plotflag)

%% plot
plotflag = 1 ; 
[p2] = definePlantModelSwift021SN018NoAero(plotflag)