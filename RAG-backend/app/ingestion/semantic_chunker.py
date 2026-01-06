"""
Semantic Chunking Service for Documentation.

Implements intelligent, meaning-preserving chunking following the canonical schema:
{
    "chunk_id": "uuid",
    "doc_id": "uuid",
    "chunk_text": "...",
    "chapter": 1,
    "section": "Introduction",
    "page": 1,
    "token_count": 190
}
"""

import logging
import re
import uuid
from typing import List, Dict, Any, Optional
from pathlib import Path
import frontmatter
import tiktoken
from app.utils.tokens import count_tokens

logger = logging.getLogger(__name__)


def safe_print(*args, **kwargs):
    """Print with fallback for Unicode encoding errors on Windows."""
    try:
        print(*args, **kwargs)
    except UnicodeEncodeError:
        # Replace common emojis with ASCII equivalents
        new_args = []
        for arg in args:
            if isinstance(arg, str):
                arg = (arg.replace('✓', '[OK]')
                         .replace('✗', '[X]')
                         .replace('→', '->'))
            new_args.append(arg)
        print(*new_args, **kwargs)


class SemanticChunker:
    """
    Semantic document chunker that preserves meaning and follows strict schema.

    Chunks by logical sections while maintaining:
    - Complete ideas and coherent sections
    - Target token range of 150-300 tokens
    - Proper metadata extraction from frontmatter
    """

    def __init__(
        self,
        min_tokens: int = 250,      # Increased from 150 to reduce number of chunks
        max_tokens: int = 600,      # Increased from 300 to reduce number of chunks
        overlap_tokens: int = 100,  # Increased from 50 to provide better context
        encoding_model: str = "cl100k_base"
    ):
        """
        Initialize semantic chunker.

        Args:
            min_tokens: Minimum tokens per chunk (default: 250 to reduce API calls)
            max_tokens: Maximum tokens per chunk (default: 600 to reduce API calls)
            overlap_tokens: Token overlap between chunks for context (default: 100)
            encoding_model: Tiktoken encoding model
        """
        self.min_tokens = min_tokens
        self.max_tokens = max_tokens
        self.overlap_tokens = overlap_tokens
        self.encoding_model = encoding_model


    def extract_metadata(self, file_path: Path) -> Dict[str, Any]:
        """
        Extract metadata from MDX frontmatter.

        Args:
            file_path: Path to MDX file

        Returns:
            Dictionary with extracted metadata
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            post = frontmatter.load(f)

        metadata = {
            'chapter': post.get('chapter_number', self._extract_chapter_from_path(file_path)),
            'title': post.get('title', file_path.stem.replace('-', ' ').title()),
            'description': post.get('description', ''),
            'sidebar_position': post.get('sidebar_position', 0),
            'module_number': post.get('module_number', 0),
            'content': post.content,
            'file_path': str(file_path)
        }

        return metadata

    def _extract_chapter_from_path(self, file_path: Path) -> int:
        """Extract chapter number from file path."""
        match = re.search(r'chapter(\d+)', str(file_path))
        if match:
            return int(match.group(1))
        return 0

    def split_into_sections(self, content: str) -> List[Dict[str, str]]:
        """
        Split content into logical sections based on markdown headers.

        Args:
            content: Markdown content

        Returns:
            List of sections with titles and content
        """
        sections = []

        # Split by headers (## or ###)
        # Pattern: ## Section Title or ### Subsection Title
        pattern = r'^(#{2,3})\s+(.+)$'

        lines = content.split('\n')
        current_section = None
        current_content = []
        current_level = 0

        for line in lines:
            header_match = re.match(pattern, line)

            if header_match:
                # Save previous section
                if current_section:
                    sections.append({
                        'title': current_section,
                        'content': '\n'.join(current_content).strip(),
                        'level': current_level
                    })

                # Start new section
                current_level = len(header_match.group(1))
                current_section = header_match.group(2).strip()
                current_content = [line]  # Include header in content
            else:
                if current_section:
                    current_content.append(line)
                else:
                    # Content before first header
                    if not current_section:
                        current_section = "Introduction"
                        current_level = 2
                    current_content.append(line)

        # Add last section
        if current_section:
            sections.append({
                'title': current_section,
                'content': '\n'.join(current_content).strip(),
                'level': current_level
            })

        return sections

    def chunk_section(self, section_text: str, section_title: str) -> List[str]:
        """
        Chunk a section into smaller pieces if needed, preserving meaning.

        Args:
            section_text: Section content
            section_title: Section title for context

        Returns:
            List of chunks
        """
        token_count = count_tokens(section_text, self.encoding_model)

        # If section is within target range, return as is
        if self.min_tokens <= token_count <= self.max_tokens:
            return [section_text]

        # If section is too small, return as is (will be merged later)
        if token_count < self.min_tokens:
            return [section_text]

        # Section is too large, split by paragraphs
        chunks = []
        paragraphs = section_text.split('\n\n')
        current_chunk = f"## {section_title}\n\n" if not section_text.startswith('#') else ""
        current_tokens = self.count_tokens(current_chunk)

        for para in paragraphs:
            para = para.strip()
            if not para:
                continue

            para_tokens = self.count_tokens(para)

            # If adding this paragraph exceeds max, save current chunk
            if current_tokens + para_tokens > self.max_tokens and current_chunk:
                chunks.append(current_chunk.strip())
                current_chunk = f"## {section_title}\n\n{para}\n\n"
                current_tokens = self.count_tokens(current_chunk)
            else:
                current_chunk += para + "\n\n"
                current_tokens += para_tokens

        # Add remaining chunk
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def create_chunk_object(
        self,
        chunk_text: str,
        doc_id: str,
        chapter: int,
        section: str,
        page: int
    ) -> Dict[str, Any]:
        """
        Create a chunk object following the canonical schema.

        Args:
            chunk_text: The chunk content
            doc_id: Document UUID
            chapter: Chapter number
            section: Section title
            page: Page/order number

        Returns:
            Chunk object matching schema
        """
        return {
            "chunk_id": str(uuid.uuid4()),
            "doc_id": doc_id,
            "chunk_text": chunk_text,
            "chapter": chapter,
            "section": section,
            "page": page,
            "token_count": self.count_tokens(chunk_text)
        }

    def process_document(
        self,
        file_path: Path,
        doc_id: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Process a complete document into semantic chunks.

        Args:
            file_path: Path to MDX file
            doc_id: Optional document UUID (generated if not provided)

        Returns:
            List of chunk objects following canonical schema
        """
        # Generate doc_id if not provided
        if not doc_id:
            doc_id = str(uuid.uuid4())

        # Extract metadata
        metadata = self.extract_metadata(file_path)

        # Split into sections
        sections = self.split_into_sections(metadata['content'])

        # Process each section into chunks
        all_chunks = []
        page_counter = 1

        for section in sections:
            section_chunks = self.chunk_section(
                section['content'],
                section['title']
            )

            for chunk_text in section_chunks:
                chunk = self.create_chunk_object(
                    chunk_text=chunk_text,
                    doc_id=doc_id,
                    chapter=metadata['chapter'],
                    section=section['title'],
                    page=page_counter
                )
                all_chunks.append(chunk)
                page_counter += 1

        return all_chunks

    def process_directory(
        self,
        docs_path: Path,
        doc_id_mapping: Optional[Dict[str, str]] = None
    ) -> List[Dict[str, Any]]:
        """
        Process all documents in a directory.

        Args:
            docs_path: Path to documentation directory
            doc_id_mapping: Optional mapping of file paths to doc_ids
                           (for maintaining consistency across runs)

        Returns:
            List of all chunks from all documents
        """
        if doc_id_mapping is None:
            doc_id_mapping = {}

        all_chunks = []

        # Find all MDX and MD files
        md_files = list(docs_path.glob('**/*.md'))
        mdx_files = list(docs_path.glob('**/*.mdx'))
        all_files = md_files + mdx_files

        logger.info(f"Found {len(all_files)} documentation files")

        for file_path in sorted(all_files):
            # Get or generate doc_id for this file
            file_key = str(file_path.relative_to(docs_path))
            doc_id = doc_id_mapping.get(file_key, str(uuid.uuid4()))
            doc_id_mapping[file_key] = doc_id

            logger.info(f"Processing: {file_path.name}")

            try:
                chunks = self.process_document(file_path, doc_id)
                all_chunks.extend(chunks)
                logger.info(f"  → Generated {len(chunks)} chunks")
            except Exception as e:
                logger.error(f"  ✗ Error processing {file_path.name}: {e}")
                continue

        logger.info(f"Total chunks generated: {len(all_chunks)}")
        return all_chunks


def validate_chunk_schema(chunk: Dict[str, Any]) -> bool:
    """
    Validate that a chunk follows the canonical schema.

    Args:
        chunk: Chunk object to validate

    Returns:
        True if valid, False otherwise
    """
    required_fields = [
        'chunk_id', 'doc_id', 'chunk_text',
        'chapter', 'section', 'page', 'token_count'
    ]

    # Check all required fields exist
    for field in required_fields:
        if field not in chunk:
            logger.error(f"Missing field: {field}")
            return False

    # Validate types
    if not isinstance(chunk['chunk_id'], str):
        logger.error("chunk_id must be string")
        return False

    if not isinstance(chunk['doc_id'], str):
        logger.error("doc_id must be string")
        return False

    if not isinstance(chunk['chunk_text'], str):
        logger.error("chunk_text must be string")
        return False

    if not isinstance(chunk['chapter'], int):
        logger.error("chapter must be int")
        return False

    if not isinstance(chunk['section'], str):
        logger.error("section must be string")
        return False

    if not isinstance(chunk['page'], int):
        logger.error("page must be int")
        return False

    if not isinstance(chunk['token_count'], int):
        logger.error("token_count must be int")
        return False

    return True
