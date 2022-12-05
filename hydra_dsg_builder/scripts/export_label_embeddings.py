#!/usr/bin/env python3
"""Quick script to export label embeddings."""
import gensim.models as gm
import numpy as np
import pathlib
import yaml
import click


@click.command()
@click.argument("word2vec_path", type=click.Path(exists=True))
@click.argument("name_mappings", type=click.Path(exists=True))
@click.argument("output_path")
@click.option("-s", "--embedding_size", type=int, default=300)
def main(word2vec_path, name_mappings, output_path, embedding_size):
    """Export embeddings to yaml."""
    word2vec_path = pathlib.Path(word2vec_path).expanduser().absolute()
    name_mappings = pathlib.Path(name_mappings).expanduser().absolute()
    output_path = pathlib.Path(output_path).expanduser().absolute()

    with name_mappings.open("r") as fin:
        name_config = yaml.load(fin.read(), Loader=yaml.SafeLoader)

    click.secho("loading word2vec...", fg="green")
    word2vec = gm.KeyedVectors.load_word2vec_format(word2vec_path, binary=True)
    click.secho("loaded word2vec!", fg="green")

    embeddings = {}
    click.secho("exporting labels:", fg="green")
    for info in name_config["label_names"]:
        label = info["label"]
        name = info["name"]
        vec = np.zeros(embedding_size)
        words = [x for x in name.split("_") if x != "of"]
        words = [x for x in words if x in word2vec]
        click.secho(f"  - {label}: name={name}, words={words}", fg="green")

        if len(words):
            vec = np.mean([word2vec[x] for x in words], axis=0)
        else:
            click.secho(f"empty word list for {label}!", fg="red")
            vec = np.zeros(embedding_size)

        embeddings[label] = [float(x) for x in vec.astype(np.float32).tolist()]

    with output_path.open("w") as fout:
        # we could use yaml.dump, but the formatting helps
        fout.write("---\n")
        fout.write("embeddings:\n")
        for label, values in embeddings.items():
            fout.write(f"  - {{label: {label}, values: {values}}}\n")


if __name__ == "__main__":
    main()
