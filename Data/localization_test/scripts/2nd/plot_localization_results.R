library(tidyverse)

setwd("/home/okamoto/Projects/beemic/Data/localization_test/")

calc_point <- function(d1, d2) {
  d1r <- d1 / 180 * pi
  d2r <- d2 / 180 * pi
  if (tan(d1r) == tan(d2r)) {
    return(c(NA, NA))
  }
  f <- function(x) {
    res = x * (tan(d2r) - tan(d1r)) + 5 * (tan(d2r) + tan(d1r))
    return(res)
  }
  x <- try(uniroot(f, c(-1000, 1000))$root, silent = TRUE)
  if (class(x) == "try-error") {
    return(c(NA, NA))
  }
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

df <- read_csv("scripts/2nd/doa.csv") %>%
  pivot_wider(id_cols = c(condition, sound), names_from = mic, values_from = doa) %>%
  mutate(
    micR = as.integer(micR),
    micL = as.integer(micL)
  ) %>%
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
    pred_x = calc_point(micR, micL)[1],
    pred_y = calc_point(micR, micL)[2]
  ) %>% 
  mutate(
    error = sqrt((true_x - pred_x)^2 + (true_y - pred_y)^2)
  ) %>%
  mutate(
    true_micR = calc_deg(5, 0, true_x, true_y),
    true_micL = calc_deg(-5, 0, true_x, true_y)
  ) %>%
  rename(
    pred_micR = micR,
    pred_micL = micL
  ) %>%
  pivot_longer(
    cols = c(true_x, pred_x, true_y, pred_y, 
             true_micR, true_micL, pred_micR, pred_micL), 
    names_to = c("type", ".value"), 
    names_sep = 5) %>%
      mutate(type = str_remove(type, "_"))

p_degree <- df %>%
  mutate(
    species = case_when(
      sound == "aburazemi.wav" ~ "Large Brown Cicada",
      sound == "amagaeru.wav" ~ "Tree Frog",
      sound == "higurashi.wav" ~ "Evening Cicada",
      sound == "hikigaeru.wav" ~ "Common Toad",
      sound == "karasu.wav" ~ "Large-billed Crow",
      sound == "shijukara.wav" ~ "Japanese Tit"
    )
  ) %>%
  mutate(
    species = factor(species, levels = c(
      "Large Brown Cicada",
      "Evening Cicada",
      "Tree Frog",
      "Common Toad",
      "Large-billed Crow",
      "Japanese Tit"
    ))
  ) %>%
  pivot_wider(id_cols = c(species, degree, distance), 
              names_from = c(type), values_from = c(x, y, micR, micL)) %>%
  mutate(
    mic_error = (
      sqrt((micR_pred - micR_true)^2) + 
      sqrt((micL_pred - micL_true)^2)
    ) / 2
  ) %>%
  select(species, degree, distance, mic_error) %>%
  mutate(distance = as.factor(distance)) %>%
  ggplot(aes(x = distance, y = mic_error)) +
  geom_boxplot() +
  facet_wrap(~species) +
  labs(
    x = "Distance (m)",
    y = "Estimation Error (degree)"
  ) + 
  theme(
    title = element_text(size = 14),
    strip.text = element_text(size = 14)
  )

p_degree
ggsave("figures/localization_degree.png", p_degree, width = 9, height = 6)
  
df %>%
  pivot_wider(id_cols = c(sound, degree, distance), names_from = c(type), values_from = c(x, y, micR, micL)) %>%
  mutate(
    micR_error = sqrt((micR_pred - micR_true)^2),
    micL_error = sqrt((micL_pred - micL_true)^2)
  ) %>%
  select(micR_error, micL_error) %>%
  summarise_all(mean)

df %>%
  filter(is.na(error))

df %>% 
  arrange(desc(error)) %>%
  View()

df %>%
  group_by(degree, distance) %>%
  summarise(error = mean(error))

df %>%
  group_by(distance) %>%
  summarise(error = mean(error))

df  %>%
  ggplot(aes(x = x, y = y, color = sound, shape = type)) +
  geom_point(size = 3, alpha = 0.5) +
  xlim(-10, 30) +
  ylim(0, 40) +
  facet_wrap(degree~distance)

df  %>%
  ggplot(aes(x = x, y = y, color = sound, shape = type)) +
  geom_point(size = 3, alpha = 0.5) +
  xlim(-10, 30) +
  ylim(0, 40)

df %>%
  mutate(distance = distance) %>%
  ggplot(aes(x = distance, y = error, color = sound)) +
  geom_line() +
  facet_wrap(~degree)

df %>%
  pivot_wider(id_cols = c(sound, degree, distance), names_from = c(type), values_from = c(x, y, micR, micL)) %>%
  mutate(
    micR_bias = micR_pred - micR_true,
    micL_bias = micL_pred - micL_true,
    micR_error = sqrt((micR_pred - micR_true)^2),
    micL_error = sqrt((micL_pred - micL_true)^2)
  ) %>%
  select(degree, distance, micR_bias, micL_bias, micR_error, micL_error) %>%
  #group_by(degree, distance) %>%
  summarize_all(mean)

# 角度のバイアスを引いてみる
biases <- df %>%
  pivot_wider(id_cols = c(sound, degree, distance), names_from = c(type), values_from = c(x, y, micR, micL)) %>%
  mutate(
    micR_bias = micR_pred - micR_true,
    micL_bias = micL_pred - micL_true,
    micR_error = sqrt((micR_pred - micR_true)^2),
    micL_error = sqrt((micL_pred - micL_true)^2)
  ) %>%
  select(micR_bias, micL_bias) %>%
  summarize_all(mean)

df2 <- df %>%
  pivot_wider(id_cols = c(sound, degree, distance), names_from = c(type), values_from = c(x, y, micR, micL)) %>%
  select(-c(x_pred, y_pred)) %>%
  mutate(
    micR_pred = micR_pred - biases$micR_bias,
    micL_pred = micL_pred - biases$micL_bias
  ) %>%
  rowwise() %>%
  mutate(
    x_pred = calc_point(micR_pred, micL_pred)[1],
    y_pred = calc_point(micR_pred, micL_pred)[2]
  ) %>% 
  mutate(
    error = sqrt((x_true - x_pred)^2 + (y_true - y_pred)^2)
  ) %>%
  pivot_longer(
    cols = c(x_true, x_pred, y_true, y_pred, 
             micR_true, micL_true, micR_pred, micL_pred), 
    names_to = c(".value", "type"), 
    names_sep = -5) %>%
  mutate(type = str_remove(type, "_")) 

df2 %>%
  pivot_wider(id_cols = c(sound, degree, distance), names_from = c(type), values_from = c(x, y, micR, micL)) %>%
  mutate(
    micR_bias = micR_pred - micR_true,
    micL_bias = micL_pred - micL_true,
    micR_error = sqrt((micR_pred - micR_true)^2),
    micL_error = sqrt((micL_pred - micL_true)^2)
  ) %>%
  select(degree, distance, micR_bias, micL_bias, micR_error, micL_error) %>%
  group_by(degree, distance) %>%
  summarize_all(mean)

df2 %>%
  group_by(degree, distance) %>%
  summarise(
    error = mean(error)
  )

df2  %>%
  ggplot(aes(x = x, y = y, color = sound, shape = type)) +
  geom_point(size = 3, alpha = 0.5) +
  xlim(-10, 30) +
  facet_wrap(degree~distance)

