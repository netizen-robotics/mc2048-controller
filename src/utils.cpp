float movingAverage(float newValue, float lastAverage, int windowSize) {
    return (lastAverage * (windowSize - 1) + newValue) / windowSize;
}