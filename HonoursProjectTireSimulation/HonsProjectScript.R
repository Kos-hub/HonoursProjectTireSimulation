library(ggplot2)
library(ggpubr)

setwd("C:/Users/giana/source/repos/HonoursProjectTireSimulation/HonoursProjectTireSimulation")
filenames<-list.files(path=".", pattern="*.csv")
names<-substr(filenames,1,nchar(filenames) - 4)
listOfTables<-list()

for(i in names){
  filepath <- file.path(".", paste(i,".csv", sep=""))
  currTable<-read.csv(filepath)
  
  colnames(currTable)<-c("Slip","Force")
  ordTable<-currTable[order(-currTable$Slip), ,drop=FALSE]
  
  longPlot<-ggplot(data = ordTable, mapping = aes(x=ordTable$Slip, y=ordTable$Force, color = i)) + geom_line() + geom_point() + theme_minimal() + 
    geom_vline(aes(xintercept = 0)) +
    geom_hline(aes(yintercept = 0)) +
    labs(x = ifelse(grepl("long", tolower(i)), "Longitudinal Slip",
                    ifelse(grepl("lat", tolower(i)), "Lateral Slip (Deg)", "Lateral Slip (Deg)")),
         y = ifelse(grepl("long", tolower(i)), "Longitudinal Force (N)",
                    ifelse(grepl("lat", tolower(i)), "Lateral Force (N)",
                           ifelse(grepl("alg", tolower(i)), "Align Moment (Nm)", "Default Y-Axis Label"))))+
    scale_color_manual(values = ifelse(grepl("magic", tolower(i)), "blue",
                                       ifelse(grepl("dugoff", tolower(i)), "red",
                                              ifelse(grepl("default", tolower(i)), "purple",
                                                     ifelse(grepl("fiala", tolower(i)), "orange", "black"))))) +
    theme(legend.position = "none", 
          axis.title.x = element_blank(),
          axis.title.y = element_blank(),
          axis.text = element_text(size = 24))  # Remove the legend
  ggsave(paste(i, "Graph.jpeg", sep=""),plot = longPlot, jpeg)
}
