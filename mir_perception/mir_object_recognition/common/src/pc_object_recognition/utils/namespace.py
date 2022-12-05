class Dgcnn_Namespace:
    def __init__(self, **kwargs):

        emb_dims = 1024
        dropout = 0.5
        k = 20
        self.__dict__.update(emb_dims=emb_dims, dropout=dropout,k=k, **kwargs)