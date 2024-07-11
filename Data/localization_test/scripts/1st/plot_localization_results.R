library(tidyverse)

setwd("~/Projects/SpresenseOutdoorMicArray/localization_test/")

calc_point <- function(d1, d2) {
  d1r <- d1 / 180 * pi
  d2r <- d2 / 180 * pi
  f <- function(x) {
    res = x * (tan(d2r) - tan(d1r)) + 5 * (tan(d2r) + tan(d1r)) 
    return(res)
  }
  x <- uniroot(f, c(-100, 100))$root
  y <- tan(d1r) * x - 5 * tan(d1r)
  return(c(x, y))
}

calc_deg <- function(xm, ym, xs, ys) { # x_mic, x_source
  # マイクの位置に原点を移す
  x = xs - xm
  y = ys - ym
  # 角度計算
  deg = atan(y / x) * 180 / pi
  if (deg < 0) {
    deg <- 180 + deg
  }
  return(deg)
}

df <- read_csv("doa.csv") %>%
  pivot_wider(id_cols = c(condition, sound), names_from = mic, values_from = doa) %>%
  mutate(
    degree = as.numeric(str_extract(condition, "\\d*deg") %>% str_remove("deg")),
    distance = as.numeric(str_extract(condition, "\\d*m") %>% str_remove("m")),
  ) %>%
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
  mutate(
    true_micA = calc_deg(5, 0, true_x, true_y),
    true_micB = calc_deg(-5, 0, true_x, true_y)
  ) %>%
  rename(
    pred_micA = micA,
    pred_micB = micB
  ) %>%
  pivot_longer(
    cols = c(true_x, pred_x, true_y, pred_y, 
             true_micA, true_micB, pred_micA, pred_micB), 
    names_to = c("type", ".value"), 
    names_sep = 5) %>%
  mutate(type = str_remove(type, "_")) 

df  %>%
  ggplot(aes(x = x, y = y, color = sound, shape = type)) +
  geom_point(size = 3, alpha = 0.5) +
  xlim(-10, 10) +
  facet_wrap(degree~distance)

df %>%
  pivot_wider(id_cols = c(sound, degree, distance), names_from = c(type), values_from = c(x, y, micA, micB)) %>%
  mutate(
    micA_bias = micA_pred - micA_true,
    micB_bias = micB_pred - micB_true,
    micA_error = sqrt((micA_pred - micA_true)^2),
    micB_error = sqrt((micB_pred - micB_true)^2)
  ) %>%
  select(degree, distance, micA_bias, micB_bias, micA_error, micB_error) %>%
  group_by(degree, distance) %>%
  summarize_all(mean)

df %>%
  group_by(degree, distance) %>%
  summarise(error = mean(error))

# 角度のバイアスを引いてみる
biases <- df %>%
  pivot_wider(id_cols = c(sound, degree, distance), names_from = c(type), values_from = c(x, y, micA, micB)) %>%
  mutate(
    micA_bias = micA_pred - micA_true,
    micB_bias = micB_pred - micB_true,
    micA_error = sqrt((micA_pred - micA_true)^2),
    micB_error = sqrt((micB_pred - micB_true)^2)
  ) %>%
  select(micA_bias, micB_bias) %>%
  summarize_all(mean)

df2 <- df %>%
  pivot_wider(id_cols = c(sound, degree, distance), names_from = c(type), values_from = c(x, y, micA, micB)) %>%
  select(-c(x_pred, y_pred)) %>%
  mutate(
    micA_pred = micA_pred - biases$micA_bias,
    micB_pred = micB_pred - biases$micB_bias
  ) %>%
  rowwise() %>%
  mutate(
    x_pred = calc_point(micA_pred, micB_pred)[1],
    y_pred = calc_point(micA_pred, micB_pred)[2]
  ) %>% 
  mutate(
    error = sqrt((x_true - x_pred)^2 + (y_true - y_pred)^2)
  ) %>%
  pivot_longer(
    cols = c(x_true, x_pred, y_true, y_pred, 
             micA_true, micB_true, micA_pred, micB_pred), 
    names_to = c(".value", "type"), 
    names_sep = -5) %>%
  mutate(type = str_remove(type, "_")) 
  

df2 %>%
  pivot_wider(id_cols = c(sound, degree, distance), names_from = c(type), values_from = c(x, y, micA, micB)) %>%
  mutate(
    micA_bias = micA_pred - micA_true,
    micB_bias = micB_pred - micB_true,
    micA_error = sqrt((micA_pred - micA_true)^2),
    micB_error = sqrt((micB_pred - micB_true)^2)
  ) %>%
  select(degree, distance, micA_bias, micB_bias, micA_error, micB_error) %>%
  group_by(degree, distance) %>%
  summarize_all(mean)

df2 %>%
  group_by(degree, distance) %>%
  summarise(
    mean_error = mean(error),
    var_error = var(error),
    max_error = max(error)
  )

df2  %>%
  ggplot(aes(x = x, y = y, color = sound, shape = type)) +
  geom_point(size = 3, alpha = 0.5) +
  xlim(-10, 10) +
  facet_wrap(degree~distance)

