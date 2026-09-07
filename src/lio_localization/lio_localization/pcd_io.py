"""
Small, dependency-free PCD loader for fixed localization maps.

Only XYZ is returned.  ASCII, binary, and PCL's binary_compressed layout are
supported so the localization boundary does not need Open3D or Elevator-LIO's
internal relocation map loader.
"""

from pathlib import Path
import struct
from typing import Dict, List, Tuple

from ament_index_python.packages import get_package_share_directory
import numpy as np


_PCD_DTYPES = {
    ('F', 4): np.dtype('<f4'),
    ('F', 8): np.dtype('<f8'),
    ('I', 1): np.dtype('<i1'),
    ('I', 2): np.dtype('<i2'),
    ('I', 4): np.dtype('<i4'),
    ('I', 8): np.dtype('<i8'),
    ('U', 1): np.dtype('<u1'),
    ('U', 2): np.dtype('<u2'),
    ('U', 4): np.dtype('<u4'),
    ('U', 8): np.dtype('<u8'),
}


def resolve_map_path(map_path: str) -> Path:
    """Resolve an absolute path or ``package://package/relative/path`` URI."""
    value = map_path.strip()
    if not value:
        raise ValueError('map_path is empty')

    if value.startswith('package://'):
        package_and_path = value[len('package://'):].split('/', 1)
        if len(package_and_path) != 2 or not all(package_and_path):
            raise ValueError(
                'package map_path must be package://<package>/<relative-path>')
        package, relative_path = package_and_path
        path = Path(get_package_share_directory(package)) / relative_path
    else:
        path = Path(value)
        if not path.is_absolute():
            raise ValueError(
                'map_path must be absolute or use package://<package>/<relative-path>')

    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f'PCD map does not exist: {path}')
    return path


def _read_header(stream) -> Tuple[Dict[str, List[str]], int]:
    header: Dict[str, List[str]] = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError('PCD header ended before a DATA declaration')
        try:
            text = line.decode('ascii').strip()
        except UnicodeDecodeError as exc:
            raise ValueError('PCD header is not ASCII') from exc
        if not text or text.startswith('#'):
            continue
        parts = text.split()
        key = parts[0].upper()
        header[key] = parts[1:]
        if key == 'DATA':
            if len(parts) != 2:
                raise ValueError('invalid PCD DATA declaration')
            return header, stream.tell()


def _parse_layout(header: Dict[str, List[str]]):
    fields = header.get('FIELDS') or header.get('FIELD')
    if not fields:
        raise ValueError('PCD header has no FIELDS declaration')
    fields = [field.lower() for field in fields]
    try:
        sizes = [int(value) for value in header['SIZE']]
        types = [value.upper() for value in header['TYPE']]
        counts = [int(value) for value in header.get('COUNT', ['1'] * len(fields))]
    except (KeyError, ValueError) as exc:
        raise ValueError('invalid PCD SIZE/TYPE/COUNT declaration') from exc

    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError('PCD FIELDS/SIZE/TYPE/COUNT lengths differ')
    if any(count <= 0 for count in counts):
        raise ValueError('PCD field COUNT values must be positive')

    try:
        point_count = int(header.get('POINTS', ['0'])[0])
        if point_count <= 0:
            width = int(header['WIDTH'][0])
            height = int(header.get('HEIGHT', ['1'])[0])
            point_count = width * height
    except (KeyError, ValueError) as exc:
        raise ValueError('invalid PCD POINTS/WIDTH/HEIGHT declaration') from exc
    if point_count <= 0:
        raise ValueError('PCD declares zero points')

    dtypes = []
    for type_name, size in zip(types, sizes):
        try:
            dtypes.append(_PCD_DTYPES[(type_name, size)])
        except KeyError as exc:
            raise ValueError(
                f'unsupported PCD scalar type TYPE={type_name} SIZE={size}') from exc
    return fields, sizes, counts, dtypes, point_count


def _extract_ascii(stream, fields, counts, point_count):
    values = np.loadtxt(stream, dtype=np.float64, ndmin=2, max_rows=point_count)
    expected_columns = sum(counts)
    if values.shape != (point_count, expected_columns):
        raise ValueError(
            f'PCD ASCII payload shape is {values.shape}, expected '
            f'({point_count}, {expected_columns})')
    offsets = {}
    column = 0
    for field, count in zip(fields, counts):
        offsets[field] = (column, count)
        column += count
    return {
        name: values[:, offsets[name][0]]
        for name in ('x', 'y', 'z')
    }


def _extract_binary(stream, fields, counts, dtypes, point_count):
    dtype_fields = []
    for index, (field, count, dtype) in enumerate(zip(fields, counts, dtypes)):
        # Prefix names to keep NumPy's structured dtype valid even for an
        # unusual PCD containing duplicate field labels.
        name = f'{index}_{field}'
        dtype_fields.append((name, dtype) if count == 1 else (name, dtype, (count,)))
    record_dtype = np.dtype(dtype_fields, align=False)
    expected_size = record_dtype.itemsize * point_count
    payload = stream.read(expected_size)
    if len(payload) != expected_size:
        raise ValueError(
            f'PCD binary payload is truncated: {len(payload)} of {expected_size} bytes')
    records = np.frombuffer(payload, dtype=record_dtype, count=point_count)
    result = {}
    for name in ('x', 'y', 'z'):
        index = fields.index(name)
        values = records[f'{index}_{name}']
        result[name] = values if counts[index] == 1 else values[:, 0]
    return result


def _lzf_decompress(data: bytes, expected_size: int) -> bytes:
    """Decompress the LZF stream used by PCL binary_compressed PCD files."""
    output = bytearray(expected_size)
    input_index = 0
    output_index = 0
    data_size = len(data)

    while input_index < data_size:
        control = data[input_index]
        input_index += 1
        if control < 32:
            length = control + 1
            if input_index + length > data_size or output_index + length > expected_size:
                raise ValueError('invalid LZF literal run in PCD payload')
            output[output_index:output_index + length] = data[
                input_index:input_index + length]
            input_index += length
            output_index += length
            continue

        length = control >> 5
        reference = output_index - ((control & 0x1f) << 8) - 1
        if length == 7:
            if input_index >= data_size:
                raise ValueError('truncated LZF length in PCD payload')
            length += data[input_index]
            input_index += 1
        if input_index >= data_size:
            raise ValueError('truncated LZF reference in PCD payload')
        reference -= data[input_index]
        input_index += 1
        length += 2
        if reference < 0 or output_index + length > expected_size:
            raise ValueError('invalid LZF back-reference in PCD payload')
        for _ in range(length):
            output[output_index] = output[reference]
            output_index += 1
            reference += 1

    if output_index != expected_size:
        raise ValueError(
            f'LZF output is {output_index} bytes, expected {expected_size}')
    return bytes(output)


def _extract_binary_compressed(stream, fields, sizes, counts, dtypes, point_count):
    sizes_header = stream.read(8)
    if len(sizes_header) != 8:
        raise ValueError('PCD binary_compressed payload has no size header')
    compressed_size, uncompressed_size = struct.unpack('<II', sizes_header)
    compressed = stream.read(compressed_size)
    if len(compressed) != compressed_size:
        raise ValueError('PCD binary_compressed payload is truncated')

    expected_size = point_count * sum(
        size * count for size, count in zip(sizes, counts))
    if uncompressed_size != expected_size:
        raise ValueError(
            f'PCD uncompressed size is {uncompressed_size}, expected {expected_size}')
    payload = _lzf_decompress(compressed, uncompressed_size)

    # PCL transposes binary_compressed payloads into a field-major layout.
    result = {}
    offset = 0
    for field, size, count, dtype in zip(fields, sizes, counts, dtypes):
        scalar_count = point_count * count
        block_size = scalar_count * size
        values = np.frombuffer(
            payload, dtype=dtype, count=scalar_count, offset=offset)
        if field in ('x', 'y', 'z'):
            result[field] = values if count == 1 else values.reshape(point_count, count)[:, 0]
        offset += block_size
    return result


def load_pcd_xyz(path: Path) -> np.ndarray:
    """Load finite XYZ coordinates from a PCD without changing the source file."""
    with path.open('rb') as stream:
        header, data_offset = _read_header(stream)
        fields, sizes, counts, dtypes, point_count = _parse_layout(header)
        missing = sorted({'x', 'y', 'z'} - set(fields))
        if missing:
            raise ValueError(f'PCD map is missing XYZ fields: {missing}')
        stream.seek(data_offset)
        data_kind = header['DATA'][0].lower()
        if data_kind == 'ascii':
            xyz = _extract_ascii(stream, fields, counts, point_count)
        elif data_kind == 'binary':
            xyz = _extract_binary(stream, fields, counts, dtypes, point_count)
        elif data_kind == 'binary_compressed':
            xyz = _extract_binary_compressed(
                stream, fields, sizes, counts, dtypes, point_count)
        else:
            raise ValueError(f'unsupported PCD DATA encoding: {data_kind}')

    points = np.column_stack((xyz['x'], xyz['y'], xyz['z'])).astype(
        np.float64, copy=False)
    points = np.ascontiguousarray(points[np.all(np.isfinite(points), axis=1)])
    if points.shape[0] == 0:
        raise ValueError(f'PCD map contains no finite XYZ points: {path}')
    return points
