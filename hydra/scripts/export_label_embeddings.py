#!/usr/bin/env python3
# Copyright 2022, Massachusetts Institute of Technology.
# All Rights Reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Research was sponsored by the United States Air Force Research Laboratory and
# the United States Air Force Artificial Intelligence Accelerator and was
# accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
# and conclusions contained in this document are those of the authors and should
# not be interpreted as representing the official policies, either expressed or
# implied, of the United States Air Force or the U.S. Government. The U.S.
# Government is authorized to reproduce and distribute reprints for Government
# purposes notwithstanding any copyright notation herein.

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
