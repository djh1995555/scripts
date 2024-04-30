#!/usr/bin/env python
from ad_cloud.record.writer.mcap_writer import McapWriter
from utils.color_string import *
from rich.console import Console
from rich.progress import (
    Progress,
    TextColumn,
    TimeElapsedColumn,
    BarColumn,
    SpinnerColumn,
)
console = Console()
print = console.print

class RecordGenerator():
    def __init__(self):
        pass
    def write_record(self, reader, watch_channel, output_name, data_source:str):
        channel_name_to_channel_and_schemas = {}
        channel_and_messages = []

        # Date created before 2023/9/20 has no channel cache, we have
        # to download full record and filter channel in local.
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            BarColumn(),
            TextColumn("Read {task.completed} messages"),
            TimeElapsedColumn(),
            transient=False,
        ) as progress:
            task = progress.add_task(
                f"Reading date from {color_path(data_source)}", total=None
            )
            for idx, (schema, channel, message) in enumerate(reader.iter_raw(channels=watch_channel)):
                progress.update(task, completed=idx)
                channel_name_to_channel_and_schemas[channel.name] = [
                    channel,
                    schema,
                ]
                channel_and_messages.append((channel, message))

        print(f"Writing {len(channel_name_to_channel_and_schemas)} channels")
        writer = McapWriter()
        writer.open(output_name)
        for idx, (channel_name, (channel, schema)) in enumerate(channel_name_to_channel_and_schemas.items()):
            writer.write_channel(channel, schema)

        print(f"Writing {len(channel_and_messages)} messages to {output_name}")
        for channel, message in channel_and_messages:
            writer.write_message(channel, message)
        writer.close()