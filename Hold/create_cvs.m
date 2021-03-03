 newList = [];
for k1=1:size(Boulderingholdlists,1)
    
    newList = [newList; k1 Boulderingholdlists(k1,2) Boulderingholdlists(k1,3)];
    newList = [newList; k1 Boulderingholdlists(k1,1) Boulderingholdlists(k1,3)];
end


%xlswrite('HoldExperiments_test.xlsx',newList)