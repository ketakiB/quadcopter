% don't forget to edit names

method = 'lqg';
refmethod = '';
extra = '';

set(f1,'PaperPositionMode','auto');
print(f1,strcat(method,'_',refmethod,'_',extra,'outputs'),'-dpng','-r300');
set(f2,'PaperPositionMode','auto');
print(f2,strcat(method,'_',refmethod,'_',extra,'traj'),'-dpng','-r300');
set(f3,'PaperPositionMode','auto');
print(f3,strcat(method,'_',refmethod,'_',extra,'states'),'-dpng','-r300');
set(f4,'PaperPositionMode','auto');
print(f4,strcat(method,'_',refmethod,'_',extra,'control'),'-dpng','-r300');

set(f5,'PaperPositionMode','auto');
print(f5,strcat(method,'_',refmethod,'_',extra,'xyz'),'-dpng','-r300');
set(f6,'PaperPositionMode','auto');
print(f6,strcat(method,'_',refmethod,'_',extra,'angles'),'-dpng','-r300');