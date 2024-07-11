setwd("~/Projects/SpresenseOutdoorMicArray/localization_test/HARK")
library(tidyverse)
library(XML)

read_harkxml <- function(path) {
  doc <- xmlInternalTreeParse(path) %>%
    getNodeSet("//position")
  get_attr <- function(attr) {
    map_dbl(doc, function(d) xmlGetAttr(d, attr) %>% as.numeric) 
  }
  attrs <- c("x", "y", "z", "id") 
  attrs %>%
    map_dfc(get_attr) %>%
    `colnames<-`(attrs)
}

run_music <- function(file_name) {
  cmd <- str_c("batchflow music_noisered.n ", file_name)
  system(cmd)
  read_harkxml("sources.txt") %>%
    group_by(id) %>%
    summarize(
      n = n(),
      x = mean(x),
      y = mean(y)
    ) %>%
    arrange(desc(n)) %>%
    mutate(
      fname = file_name,
      rank = row_number(desc(n))
    )
}

files <- list.files("../recordings/separated/", full.names = T, recursive = T)
df <- files %>%
  map_dfr(run_music) 

df %>%
  write_csv("doas.csv")

# HARKで用いる座標は...x軸の正方向が正面，
# y軸の正方向が左，z軸の正方向が上になるようにしている．
calc_rad <- function(x, y) {
  y1 = x
  x1 = -y
  pi - atan(x1 / y1)
}

calc_point <- function(d1, d2) {
  f <- function(x) {
    res = x * (tan(d2) - tan(d1)) + 5 * (tan(d2) + tan(d1)) 
    return(res)
  }
  x <- uniroot(f, c(-100, 100))$root
  y <- tan(d1) * x - 5 * tan(d1)
  return(c(x, y))
}

df <- read_csv("doas.csv") %>%
  filter(id == 0) %>%
  mutate(
    degree = as.numeric(str_extract(fname, "\\d*deg") %>% str_remove("deg")),
    distance = as.numeric(str_extract(fname, "\\d*m") %>% str_remove("m")),
    mic = str_extract(fname, "mic."),
    sound = str_extract(fname, "m/.*/mic") %>% str_remove("m/") %>% str_remove("/mic"),
    rad = calc_rad(x, y)
  ) %>%
  pivot_wider(id_cols = c(sound, degree, distance), names_from = mic, values_from = rad) %>%
  arrange(degree, distance, sound) %>%
  mutate(
    true_x = sin(degree / 180 * pi) * distance,
    true_y = cos(degree / 180 * pi) * distance,
  ) %>%
  rowwise() %>%
  mutate(
    pred_x = calc_point(micA, micB)[1],
    pred_y = calc_point(micA, micB)[2]
  ) %>% 
  mutate(
    error = sqrt((true_x - pred_x)^2 + (true_y - pred_y)^2)
  ) %>%
  pivot_longer(cols = c(true_x, pred_x, true_y, pred_y), names_to = c("type", ".value"), names_sep = 5) %>%
  mutate(type = str_remove(type, "_")) 

df  %>%
  ggplot(aes(x = x, y = y, color = sound, shape = type)) +
  geom_point(size = 3, alpha = 0.5) +
  xlim(-10, 10) +
  facet_wrap(degree~distance)

df %>%
  group_by(degree, distance) %>%
  summarise(error = mean(error))

