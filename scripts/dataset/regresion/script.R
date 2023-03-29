#dataset <- read.csv('dataset.csv')
dataset <- read.csv('dataset2.csv')

lm(dataset$g1 ~ dataset$x1L + dataset$y1L + dataset$x2L + dataset$y2L + dataset$x1R + dataset$y1R + dataset$x2R + dataset$y2R)

#lm(dataset$g2 ~ dataset$x1L + dataset$y1L + dataset$x2L + dataset$y2L + dataset$x1R + dataset$y1R + dataset$x2R + dataset$y2R)

#lm(dataset$g1 ~ dataset$x1L + dataset$y1L + dataset$x2L + dataset$x1R + dataset$y1R + dataset$x2R)
#summary(model)
