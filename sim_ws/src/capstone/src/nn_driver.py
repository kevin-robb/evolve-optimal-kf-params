from nn import NeuralNetwork

def get_data():
    # TODO read in the data file
    pass

def make_and_train_nn():
    # params to set manually
    learning_rate = 0.3
    n_hidden = 5
    n_epochs = 500
    test_ratio = 0.2 #proportion of data for testing
    # params that are automatically set
    dataset = get_data()
    test_ind = int(len(dataset) * (1 - test_ratio))
    training_data = dataset[:test_ind]
    testing_data = dataset[test_ind:]
    n_inputs = len(dataset[0])-1
    n_outputs = len(set([row[-1] for row in dataset]))
    net = NeuralNetwork(learning_rate,n_inputs,n_hidden,n_outputs)
    net.train(training_data, n_epochs)
    print(net)
    predictions = net.predict_list(testing_data)
    # TODO make function to calculate the accuracy

make_and_train_nn()