import matplotlib.pyplot as plt

class RealTimePlotter:
    def __init__(self, num_components, labels=None, plot_title="Real-Time Plot", x_label="Time (s)", y_label="Value"):
        """
        Initializes the real-time plotter.
        
        Parameters:
          - num_components: number of components (curves) in your vector.
          - labels: list of labels for each curve. If not provided, default labels are used.
          - plot_title: Title of the plot.
          - x_label: Label for the x-axis.
          - y_label: Label for the y-axis.
        """
        # Set up labels
        if labels is None:
            self.labels = [f'q {i+1}' for i in range(num_components)]
        else:
            self.labels = labels

        self.num_components = num_components
        self.plot_title = plot_title
        self.x_label = x_label
        self.y_label = y_label

        # Initialize data storage
        self.time_data = []
        self.data_components = [[] for _ in range(num_components)]
        
        # Create the figure and axes
        self.fig, self.ax = plt.subplots(figsize=(10, 5))
        self.lines = []
        for i in range(num_components):
            # Create a continuous line (no markers) for each component
            line, = self.ax.plot([], [], linewidth=2, label=self.labels[i])
            self.lines.append(line)
        
        self.ax.set_xlabel(self.x_label)
        self.ax.set_ylabel(self.y_label)
        self.ax.set_title(self.plot_title)
        self.ax.legend()
        self.ax.grid(True)
        
        plt.ion()  # Enable interactive mode
        plt.show()

    def update(self, t, vector):
        """
        Updates the plot with a new time value 't' and a vector of data.
        
        Parameters:
          - t: The current time value (float).
          - vector: An iterable or a scalar value containing new data.
                    If a scalar is provided, it will be wrapped into a list.
        """
        # If the vector is a scalar, wrap it in a list.
        if not hasattr(vector, '__iter__'):
            vector = [vector]
        
        self.time_data.append(t)
        
        # Update each curve with the new data point.
        for i, val in enumerate(vector):
            if i < self.num_components:
                self.data_components[i].append(val)
                self.lines[i].set_data(self.time_data, self.data_components[i])
        
        # Rescale axes to include new data
        self.ax.relim()
        self.ax.autoscale_view()
        
        plt.draw()
        plt.pause(0.01)