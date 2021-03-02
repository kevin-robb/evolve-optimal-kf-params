from random import random
from math import exp
from typing import List
class NeuralNetwork:
    def __init__(self,learning_rate:float,n_in:int,n_hidden:int,n_out:int):
        self.learning_rate = learning_rate
        self.num_outputs = n_out
        # network is a list of layers
        self.network = []
        # each layer is a list of nodes
        hidden_layer = [Neuron(weights=[random() for i in range(n_in+1)]) for i in range(n_hidden)]
        self.network.append(hidden_layer)
        output_layer = [Neuron(weights=[random() for i in range(n_hidden+1)]) for i in range(n_out)]
        self.network.append(output_layer)

    def __str__(self) -> str:
        result = "--"
        for layer in self.network:
            for neuron in layer:
                result += str(neuron)
            result += "\n--"
        return result

    def forward_propagate(self, inputs:List[float]) -> List[float]:
        for layer in self.network:
            outputs = []
            for neuron in layer:
                neuron.transfer(inputs)
                outputs.append(neuron.output)
            inputs = outputs
        return outputs

    def backpropagate_error(self, expected:List[float]):
        for i in reversed(range(len(self.network))):
            layer = self.network[i]
            errors = []
            # handle error differently for output layer and hidden layer(s)
            if i == len(self.network) - 1: # output layer
                for j in range(len(layer)):
                    errors.append(expected[j] - layer[j].output)
            else: # hidden layer
                for j in range(len(layer)):
                    err = 0.0
                    for neuron in self.network[i+1]:
                        err += neuron.weights[j] * neuron.error
                    errors.append(err)
            # set the error on all nodes
            for n in range(len(layer)):
                layer[n].set_error(errors[n])
    
    def update_weights(self, row:List[float]):
        # we use online learning (update after every training example)
        for i in range(len(self.network)):
            if i == 0:
                # first layer. strip the label from inputs
                inputs = row[0:-1]
            else:
                # inputs are the outputs from prev layer
                inputs = [neuron.output for neuron in self.network[i-1]]
            # have each node update its weights
            for neuron in self.network[i]:
                neuron.update_weights(inputs, self.learning_rate)

    def train(self, training_data:List[List[float]], n_epoch:int):
        # train the network for n_epoch epochs
        for epoch in range(n_epoch):
            sum_error = 0
            for row in training_data:
                # get our model's predictions
                outputs = self.forward_propagate(row)
                # get the true labels (using one-hot encoding)
                expected = [0 for i in range(self.num_outputs)]
                expected[row[-1]] = 1
                sum_error += sum([(expected[i]-outputs[i])**2 for i in range(len(expected))])
                self.backpropagate_error(expected)
                self.update_weights(row)
            print(">epoch " + str(epoch) + ": error={err:.4f}".format(err=sum_error))
                
    def predict(self, row:List[float]) -> int:
        # make a prediction using the trained network
        outputs = self.forward_propagate(row)
        # undo the one-hot encoding to get back a catagorical prediction
        prediction = outputs.index(max(outputs))
        print("Predicting " + str(prediction) + ", Actual is " + str(row[-1]))
        return prediction

    def predict_list(self, rows:List[List[float]]) -> List[int]:
        predictions = []
        for row in rows:
            predictions.append(self.predict(row))
        return predictions

class Neuron:
    # each node has n_in+1 (initially random) weights,
    # one for each node in the previous layer + 1 for the bias
    def __init__(self, weights:List[float]):
        self.weights = weights
        self.output = 0
        self.error = 0

    def __str__(self) -> str:
        result = "\nweights: ["
        for w in range(len(self.weights)):
            txt = "{weight:.4f}"
            result += txt.format(weight = self.weights[w])
            if w < len(self.weights)-1: result += ","
        result += "], output: " + "{out:.4f}".format(out = self.output)
        result += ", error: " + "{err:.4f}".format(err = self.error)
        return result

    def activate(self, inputs:List[float]) -> float:
        activation = self.weights[-1] # the bias
        # weighted sum of inputs
        for i in range(len(inputs)):
            activation += self.weights[i] * inputs[i]
        return activation

    def transfer(self, inputs:List[float]):
        # sigmoid/logistic activation function (used for forward propagation)
        self.output = 1.0 / (1.0 + exp(-self.activate(inputs)))

    def transfer_derivative(self):
        # derivative of sigmoid (used for backpropagation)
        return self.output * (1.0 - self.output)

    def set_error(self,err:float):
        self.error = err * self.transfer_derivative()

    def update_weights(self, inputs:List[float], learning_rate:float):
        inputs.append(1.0) # account for bias in non-output layers
        for w in range(len(self.weights)):
            self.weights[w] += learning_rate * self.error * inputs[w]